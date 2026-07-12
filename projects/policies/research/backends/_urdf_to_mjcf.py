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

"""URDF -> MuJoCo MJCF converter for OmniSim robots.

MuJoCo can parse URDF directly via `mujoco.MjModel.from_xml_path()` for
URDFs that already use joint/link/inertial elements MuJoCo understands.
The OmniSim Spot URDF is one of these — joint axes, inertials, and the
foot-sphere collisions we added are all standard URDF that MuJoCo
accepts.

For more exotic URDFs (xacro features, non-standard tags), the open-
source `obj2mjcf` and `urdf2mjcf` tools do the conversion. We don't
ship them here; if you hit a URDF MuJoCo can't load directly, run
`pip install urdf2mjcf && urdf2mjcf path/to/robot.urdf` and point this
helper at the result.
"""
from __future__ import annotations

import os
import re
import tempfile
from pathlib import Path
from typing import Iterable, Sequence

import mujoco


_PACKAGE_RE = re.compile(r'(["\'])package://([^/"\']+)/([^"\']+)\1')


def _rewrite_package_uris(urdf_text: str, urdf_path: Path) -> str:
    """Rewrite `package://<name>/<path>` mesh URIs to filesystem paths
    MuJoCo can resolve.

    OmniSim URDFs use the ROS convention where `package://spot/meshes/foo.stl`
    means the meshes/foo.stl under the spot package dir. We assume the
    package's resource dir is the URDF's grandparent (the convention is
    `<package>/urdf/<robot>.urdf` and `<package>/meshes/...`).
    """
    package_root = urdf_path.parent.parent  # urdf/ -> package dir

    def replace(m):
        quote, pkg, rest = m.group(1), m.group(2), m.group(3)
        abs_path = (package_root / rest).resolve()
        # MuJoCo on Windows wants forward-slash paths in XML.
        return f'{quote}{abs_path.as_posix()}{quote}'

    return _PACKAGE_RE.sub(replace, urdf_text)


def load_or_convert(urdf_path: Path,
                    actuator_joints: Sequence[str] | None = None,
                    joint_limits: Sequence[tuple[float, float]] | None = None,
                    kp: float | Sequence[float] = 80.0,
                    kv: float | Sequence[float] = 2.0) -> mujoco.MjModel:
    """Load a URDF into MuJoCo, rewriting `package://` URIs to local
    paths and adding position actuators for the listed joints.

    Args:
      urdf_path: path to URDF file.
      actuator_joints: ordered list of joint names to expose as
        position actuators. URDF revolute joints don't auto-become
        MuJoCo actuators; we add them ourselves so the policy can
        drive them.
      joint_limits: per-actuator (lo, hi) ctrlrange, matching
        actuator_joints order.
      kp, kv: position actuator gains. Scalar (applied to every joint)
        or sequence (per-joint, must match actuator_joints length).
        Per-joint gains let heavy robots clamp the non-task joints
        (e.g. Atlas's arms) at stiff hold while the task joints
        (legs) use moderate gains the policy can dominate.

    Strategy:
      1. Pre-converted .mjcf.xml takes precedence.
      2. Otherwise: rewrite `package://` URIs, write rewritten URDF
         to .rewritten.urdf, load via MjSpec so we can mutate the
         model to add actuators, then compile.
    """
    urdf_path = Path(urdf_path)
    if not urdf_path.exists():
        raise FileNotFoundError(urdf_path)

    mjcf_path = urdf_path.with_suffix(".mjcf.xml")
    if mjcf_path.exists():
        return mujoco.MjModel.from_xml_path(str(mjcf_path))

    urdf_text = urdf_path.read_text()
    rewritten = _rewrite_package_uris(urdf_text, urdf_path)
    tmp = urdf_path.with_suffix(".rewritten.urdf")
    tmp.write_text(rewritten)

    try:
        spec = mujoco.MjSpec.from_file(str(tmp))
    except Exception as e:
        raise RuntimeError(
            f"MuJoCo MjSpec could not load {tmp}: {e}\n"
            f"If your URDF uses non-standard features, pre-convert with:\n"
            f"  pip install urdf2mjcf\n  urdf2mjcf {urdf_path} > {mjcf_path}\n"
        ) from e

    # Add a ground plane to the world. URDFs describe the robot only;
    # without an explicit ground geom the robot has nothing to collide
    # with in MuJoCo and falls through the world forever. The plane is
    # large enough that even a long episode can't roll off it.
    try:
        has_ground = any(
            g.type == mujoco.mjtGeom.mjGEOM_PLANE
            for g in spec.worldbody.geoms
        )
        if not has_ground:
            ground = spec.worldbody.add_geom()
            ground.name = "ground"
            ground.type = mujoco.mjtGeom.mjGEOM_PLANE
            # size = (half_x, half_y, grid_spacing); plane is at z=0,
            # normal +z by default.
            ground.size[0] = 50.0
            ground.size[1] = 50.0
            ground.size[2] = 0.1
            # Friction (tangential, torsional, rolling).
            ground.friction[0] = 1.2
            ground.friction[1] = 0.005
            ground.friction[2] = 0.0001
            ground.condim = 3
    except Exception as _ground_err:
        # If the MjSpec API doesn't expose worldbody.add_geom on this
        # mujoco version, fall back to compile-and-pray. The training
        # loop will detect the body falling and the user can pre-
        # convert with an explicit ground in the MJCF.
        pass

    # Disable robot self-collision via contype/conaffinity masking.
    # URDF mesh collisions on adjacent links inevitably overlap slightly
    # (e.g., upper leg chull and lower leg chull touch at the knee).
    # If self-collision is enabled, those overlaps push the body around
    # at simulation startup and the robot can never balance. We put all
    # robot geoms on group 2 and the ground on group 1; they only
    # collide cross-group.
    try:
        for g in spec.worldbody.geoms:
            if g.type == mujoco.mjtGeom.mjGEOM_PLANE:
                g.contype = 1
                g.conaffinity = 2
        # Recurse into body subtree (MjSpec.bodies returns flat list of
        # children of worldbody only; we need transitive descendants).
        def _walk(body):
            for g in body.geoms:
                if g.type != mujoco.mjtGeom.mjGEOM_PLANE:
                    g.contype = 2
                    g.conaffinity = 1
            for child in body.bodies:
                _walk(child)
        for top in spec.worldbody.bodies:
            _walk(top)
    except Exception:
        # MjSpec API quirks: if the iteration above isn't supported,
        # the model still loads (just with self-collision enabled).
        pass

    # Add a free joint on the root body so the base can translate +
    # rotate freely (URDF's base_link is otherwise fixed in MuJoCo).
    try:
        root_body = spec.worldbody.first_body()
        if root_body is not None:
            existing_joints = list(root_body.joints)
            has_free = any(j.type == mujoco.mjtJoint.mjJNT_FREE
                           for j in existing_joints)
            if not has_free:
                root_body.add_freejoint()
    except Exception:
        # MjSpec API surface varies across mujoco versions; if the
        # above isn't available, fall back to letting the world
        # contain the body without a free joint. The training loop
        # will still work but the body won't translate, which is
        # useful for stand-only debug.
        pass

    # Add position actuators for each named joint.
    if actuator_joints:
        # Resolve per-joint gains. Scalar gains apply to all actuators;
        # sequence gains map 1:1 to actuator_joints.
        def _per_joint(g, n):
            if hasattr(g, "__len__") and not isinstance(g, str):
                if len(g) != n:
                    raise ValueError(
                        f"per-joint gain sequence length {len(g)} != "
                        f"actuator count {n}"
                    )
                return [float(x) for x in g]
            return [float(g)] * n

        kp_seq = _per_joint(kp, len(actuator_joints))
        kv_seq = _per_joint(kv, len(actuator_joints))

        for i, joint_name in enumerate(actuator_joints):
            joint = spec.joint(joint_name) if hasattr(spec, "joint") else None
            if joint is None:
                # Older API: look by attribute.
                for j in spec.joints:
                    if j.name == joint_name:
                        joint = j
                        break
            if joint is None:
                raise RuntimeError(f"joint {joint_name!r} not found in URDF")
            act = spec.add_actuator()
            act.name = f"{joint_name}_motor"
            act.target = joint_name
            act.trntype = mujoco.mjtTrn.mjTRN_JOINT
            act.gaintype = mujoco.mjtGain.mjGAIN_FIXED
            act.biastype = mujoco.mjtBias.mjBIAS_AFFINE
            act.gainprm[0] = kp_seq[i]
            act.biasprm[1] = -kp_seq[i]
            act.biasprm[2] = -kv_seq[i]
            if joint_limits is not None and i < len(joint_limits):
                lo, hi = joint_limits[i]
                act.ctrllimited = True
                act.ctrlrange[0] = lo
                act.ctrlrange[1] = hi

    # MJX throughput: MuJoCo's default solver iterations (100/50) are
    # tuned for offline accuracy and cripple GPU-batched training — each
    # physics step runs up to 100 serial solver passes, which both slows
    # the step and leaves the GPU under-utilized. Brax/MJX humanoid
    # setups run ~10/8 with no loss of training stability. Env-tunable so
    # a robot that needs stiffer contacts can raise them; using this path
    # for both MJX training and the MuJoCo eval keeps their physics
    # consistent (good for sim-to-deploy parity).
    try:
        spec.option.iterations = int(os.environ.get("OMNISIM_MJX_SOLVER_ITER", "10"))
        spec.option.ls_iterations = int(os.environ.get("OMNISIM_MJX_LS_ITER", "8"))
    except Exception:
        pass

    return spec.compile()
