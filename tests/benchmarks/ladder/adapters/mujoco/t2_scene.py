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

"""**The three T2 descriptions -> one MuJoCo scene an arm can work in.**

T2's container ships *"one arm description, one object description, one
container description, and nothing else"* (``tasks/T2_transfer/meta.json``):
no ground, no table, no placement, no actuator, no controller. Bringing those
three bodies into a scene is part of the capability under test, so this module
is the honest record of what that costs on MuJoCo -- every edit recorded as a
cited :class:`~ladder.adapters.mujoco.model_build.BuildStep`, exactly as the T1
builder does for the Husky.

**It is a scripted fixture, not a ladder cell.** A cell is an autonomous agent
given one sentence (``capability-ladder-plan.md`` §2). What this proves is that
the task is *achievable* -- which the container's own ``PROVENANCE.txt`` records
as unproven on any simulator, and which the plan says must be settled before
the freeze or the tier measured our asset rather than anybody's agent.

Four things bite, and three of them are silent
----------------------------------------------

**1. With MuJoCo's URDF defaults, NONE of the three names survives.** Measured
on 3.8.1: ``compiler/fusestatic`` defaults to **true for URDF**, so a link with
no joint is absorbed into the world body. ``block.urdf`` and ``bin.urdf`` are
single-root descriptions with no joint at all, so each compiles to **zero
bodies** -- their geometry becomes world geometry and ``mj_name2id(...,
"block")`` returns −1. The arm keeps its articulated links and loses ``base``.
Since T2's roles are resolved **by name** (``meta.json`` → ``roles``), a scene
built on the defaults is one the grader must refuse. ``fusestatic="false"`` is
the fix and it is not optional here.

**2. The object needs a joint to the world, or it is scenery.** Same rule from
the other side: a URDF root link with no joint is welded. ``block.urdf``
describes a 0.2 kg cube and MuJoCo will happily weld it to the world for ever.
It gets URDF's own ``<joint type="floating">`` from a dummy ``world`` link --
the same in-format fix the T1 builder uses on the Husky chassis, verified there
against 3.8.1.

**3. ⚠ ANCHORING THE ARM'S PEDESTAL JAMS ITS FIRST AXIS.** The loudest finding
here, and it is a *contact-filter* effect rather than a geometry error. The
shipped arm's ``shoulder`` cylinder spans z ∈ [0.14, 0.26] while its ``base``
pedestal spans z ∈ [0, 0.15]: the two **overlap by 10 mm by design**, which is
ordinary URDF practice because a simulator filters contacts between a body and
its own parent. MuJoCo does too -- *except* that its filter is written in terms
of **weld groups**, and it deliberately does not apply when the parent is the
world body. Weld the pedestal down (no joint) and it joins the world's weld
group, the filter stops applying, and the pedestal starts colliding with the
shoulder it is bolted to. Measured: ``joint_1`` saturates at its full **150
N·m** and moves **0.02 rad in 28 s** -- the arm cannot turn, with no warning
anywhere. The scene therefore gives the pedestal a **floating joint** and lets
its 40 kg hold it down, which is what ``meta.json`` →
``container.no_fixed_base_convention`` says the pedestal is for. (An
``<exclude>`` pair between ``base`` and ``shoulder`` is the other legitimate
fix and is not used here, because the floating pedestal is the shipped intent.)

**4. A friction pinch needs the friction model configured.** With MuJoCo's
defaults (pyramidal cone, ``impratio`` 1, no noslip pass) the 0.2 kg cube
**creeps down through the jaws at ≈3.5 mm/s** under a 30 N squeeze -- 42 mm over
a 12 s carry, which is a T2.3 failure by a factor of two on a grip that is
nowhere near slipping in any physical sense. It is the tangential-softness
artifact MuJoCo's own documentation points at, and its own documented answers
fix it completely (0.0 mm of creep over the same carry): ``cone="elliptic"``,
``impratio=20`` and five ``noslip_iterations``. These are **solver settings, not
grasp tuning** -- no contact parameter, geometry, mass or commanded width was
touched -- but they are exactly the kind of undiscoverable pin the plan predicts
an agent will miss (§2 T2, on OmniSim's own ``newtonSolver "mujoco"`` pin), so
they are recorded as steps rather than buried in a default.

The layout, and why these numbers
---------------------------------

Everything is placed at one working radius so a transfer is a yaw of the first
axis plus a lift::

    ground   a 6 x 6 m slab, top at z = 0            (URDF cannot express one)
    table    0.40 x 0.44 m, top at z = 0.40, centred at radius 0.44
    block    resting on the table at radius 0.44, centre z = 0.425
    bin      on the ground at radius 0.44, 90 degrees round from the block
    arm      at the origin, pedestal free, resting on the ground

**Why 0.44 m and not the 0.9 m of reach the container advertises.** Reach is
not the binding constraint; **wrist range is**. To hold the gripper vertical the
three pitching axes must sum to π (``joint_2 + joint_3 + joint_5 = π``), and
``joint_5`` is limited to ±2.0 rad, so a straight-down grasp requires
``joint_2 + joint_3 ≥ 1.14 rad`` -- the arm must be *folded*. At radius 0.50 and
the pre-grasp height used here that sum falls to 1.14 and the IK runs out of
joint 5; at 0.44 it has 0.07 rad in hand. So the usable straight-down workspace
is a good deal smaller than the advertised reach, and that is a property of the
shipped description, recorded here rather than discovered again.
"""

from __future__ import annotations

import json
import math
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from ladder.adapters.mujoco.model_build import (  # noqa: E402
    BuildStep, DOC_COMPILER, DOC_URDF_EXTENSIONS, urdf_declared_mass)

# --- citations ---------------------------------------------------------------

DOC_ATTACH = (
    "MuJoCo model-editing API, MjSpec.attach (mujoco.readthedocs.io, "
    "Model Editing / attach): 'attach' merges a child spec into a parent at a "
    "frame or site. The prefix/suffix arguments are OPTIONAL and are left EMPTY "
    "here on purpose -- T2 resolves its three roles BY NAME from the task file, "
    "so a namespacing prefix would produce bodies carrying none of the declared "
    "names and the grader would refuse the cell rather than grade the wrong "
    "body (tasks/T2_transfer/meta.json -> roles.open_question_for_the_freeze)")
DOC_CONTACT_FILTER = (
    "MuJoCo Computation chapter, 'Collision detection' "
    "(mujoco.readthedocs.io/en/stable/computation/index.html): a candidate geom "
    "pair is discarded when the two bodies are in the same weld group, or when "
    "one body is the PARENT of the other and mjDSBL_FILTERPARENT is not set -- "
    "'except if the parent is the world body'. A pedestal welded to the world "
    "IS the world's weld group, so the parent exemption fires and the arm's own "
    "pedestal starts colliding with the shoulder bolted to it")
DOC_CONE = (
    "MuJoCo Computation chapter, 'Solver parameters' and the XML reference for "
    "option/cone, option/impratio and option/noslip_iterations: the elliptic "
    "cone is the physically accurate friction cone, impratio raises the "
    "frictional impedance relative to the normal one ('increase it to prevent "
    "slip in grasping'), and the noslip solver is a post-pass that removes the "
    "residual drift the soft-constraint formulation leaves in place")
DOC_URDF_FLOATING = (
    "URDF's own joint type='floating', honoured by MuJoCo's URDF parser "
    "(verified on 3.8.1 in this column's T1 bring-up, BRINGUP.md 3.2 stage 11). "
    "A URDF root link has no joint to the world and MuJoCo welds a jointless "
    "body to its parent")

# --- the layout, in metres ---------------------------------------------------

WORK_RADIUS = 0.44        # everything the arm touches sits at this radius
TABLE_TOP_Z = 0.40
TABLE_HALF = (0.20, 0.22)
GROUND_HALF = 3.0
GROUND_THICK = 0.05
BIN_BEARING_RAD = math.pi / 2.0    # 90 degrees round from the block

# Physics settings the pinch needs. See point 4 in the module docstring: these
# are SOLVER settings and no contact/geometry/mass parameter is touched.
TIMESTEP_S = 0.002
IMPRATIO = 20.0
SOLVER_ITERATIONS = 100
NOSLIP_ITERATIONS = 5

ARM_JOINTS = ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6")
FINGER_JOINTS = ("finger_joint_left", "finger_joint_right")

# Position-servo gains. Chosen for the shipped link masses; the force range of
# every actuator is the EFFORT THE URDF DECLARES, read out of the file rather
# than typed, so no actuator here is stronger than the description allows.
SERVO_KP = {"joint_1": 3000.0, "joint_2": 6000.0, "joint_3": 4000.0,
            "joint_4": 600.0, "joint_5": 800.0, "joint_6": 400.0,
            "finger_joint_left": 4000.0, "finger_joint_right": 4000.0}
SERVO_KV = {"joint_1": 120.0, "joint_2": 250.0, "joint_3": 180.0,
            "joint_4": 30.0, "joint_5": 40.0, "joint_6": 20.0,
            "finger_joint_left": 30.0, "finger_joint_right": 30.0}

_MUJOCO_BLOCK = (
    '  <!-- Added by the capability-ladder MuJoCo column (T2). fusestatic is\n'
    '       the load-bearing one: at its URDF default of "true" a jointless\n'
    '       root link is ABSORBED INTO THE WORLD BODY, and block.urdf and\n'
    '       bin.urdf then compile to zero named bodies. T2 resolves its roles\n'
    '       by name, so that scene is ungradeable. -->\n'
    '  <mujoco>\n'
    '    <compiler fusestatic="false" discardvisual="true"\n'
    '              balanceinertia="true" inertiafromgeom="false"/>\n'
    '  </mujoco>\n')

_FLOAT_JOINT = (
    '  <!-- Added by the capability-ladder MuJoCo column (T2): URDF\'s own\n'
    '       floating joint, so %(base)s is a free body rather than scenery. -->\n'
    '  <link name="world"/>\n'
    '  <joint name="%(base)s_float" type="floating">\n'
    '    <parent link="world"/>\n'
    '    <child link="%(base)s"/>\n'
    '    <origin rpy="0 0 0" xyz="0 0 0"/>\n'
    '  </joint>\n')


@dataclass
class T2BuildResult:
    """Everything the T2 scene build produced, and everything it changed."""

    workspace: str = ""
    scene: str = ""
    layout: dict = field(default_factory=dict)
    sources: dict = field(default_factory=dict)
    mass_kg_declared: dict = field(default_factory=dict)
    mass_kg_compiled: float = None    # the three descriptions' bodies only
    scene_mass_kg: float = None       # including the floor and the table
    bodies: int = None
    actuators: list = field(default_factory=list)
    steps: list = field(default_factory=list)
    problems: list = field(default_factory=list)

    def as_dict(self):
        return {"workspace": self.workspace, "scene": self.scene,
                "layout": dict(self.layout), "sources": dict(self.sources),
                "mass_kg_declared_by_urdf": dict(self.mass_kg_declared),
                "mass_kg_in_the_compiled_model": self.mass_kg_compiled,
                "scene_mass_kg_including_furniture": self.scene_mass_kg,
                "bodies": self.bodies, "actuators": list(self.actuators),
                "steps": [s.as_dict() for s in self.steps],
                "problems": list(self.problems),
                # The T1 builder's key, so `recording`/`evidence` read one
                # shape for both rungs.
                "urdf_source": self.sources.get("arm", ""),
                "robot_name": "arm"}


def layout(radius=WORK_RADIUS):
    """Where the three bodies go, in world metres. Written into the run."""
    bearing = BIN_BEARING_RAD
    return {
        "work_radius_m": float(radius),
        "table_top_z_m": TABLE_TOP_Z,
        "table_half_m": list(TABLE_HALF),
        "ground_top_z_m": 0.0,
        "arm_xy_m": [0.0, 0.0],
        "block_xy_m": [float(radius), 0.0],
        "bin_xy_m": [float(radius) * math.cos(bearing),
                     float(radius) * math.sin(bearing)],
        "bin_bearing_rad": float(bearing),
        "why": ("one working radius, so a transfer is a yaw of joint_1 plus a "
                "lift; 0.44 m because a STRAIGHT-DOWN grasp needs joint_2 + "
                "joint_3 >= 1.14 rad (joint_5 is limited to +/-2.0 and the "
                "three pitches must sum to pi), which the arm runs out of at "
                "0.50 m -- wrist range binds well before reach does"),
    }


def prepare_urdf(src, dest, *, floating_base=None):
    """Copy one shipped description and insert MuJoCo's own extension block.

    Never edits the container in place: the descriptions are the fixture every
    later run reads.
    """
    text = Path(src).read_text(encoding="utf-8")
    add = _MUJOCO_BLOCK
    if floating_base:
        add += _FLOAT_JOINT % {"base": floating_base}
    out = re.sub(r"(<robot\b[^>]*>)", lambda m: m.group(1) + "\n" + add,
                 text, count=1)
    Path(dest).write_text(out, encoding="utf-8")
    # MuJoCo's URDF path is NOT schema-checked (its own docs say so), and it
    # has compiled files a strict reader rejects. A file only one of the two
    # readers accepts is a file whose provenance nobody can check.
    ET.fromstring(out)
    return Path(dest)


def build_t2_scene(container_dir, workspace, *, radius=WORK_RADIUS):
    """The three descriptions in, one driveable MJCF scene out. Never raises.

    ``container_dir`` is ``tasks/T2_transfer/container``. Returns a
    :class:`T2BuildResult`; anything that went wrong is in ``problems`` and the
    scene path is left empty.
    """
    import mujoco

    res = T2BuildResult(workspace=str(workspace))
    src = Path(container_dir)
    ws = Path(workspace)
    ws.mkdir(parents=True, exist_ok=True)
    pkg = ws / "container"
    if pkg.exists():
        shutil.rmtree(pkg)
    shutil.copytree(src, pkg)
    res.steps.append(BuildStep(
        "copy", "copied the three shipped descriptions into the run workspace",
        "the ladder ships the task as files; editing them in place would "
        "mutate the fixture every later run reads", detail=str(pkg)))

    parts = {"arm": pkg / "bench_arm/urdf/bench_arm.urdf",
             "block": pkg / "props/urdf/block.urdf",
             "bin": pkg / "props/urdf/bin.urdf"}
    for name, p in parts.items():
        if not p.is_file():
            res.problems.append("the container has no %s description at %s"
                                % (name, p))
            return res
        res.sources[name] = str(src / p.relative_to(pkg))
        res.mass_kg_declared[name] = urdf_declared_mass(p)

    # -- step 1: MuJoCo's URDF extension block, on all three ---------------
    try:
        edited = {
            "arm": prepare_urdf(parts["arm"], ws / "arm.urdf",
                                floating_base="base"),
            "block": prepare_urdf(parts["block"], ws / "block.urdf",
                                  floating_base="block"),
            "bin": prepare_urdf(parts["bin"], ws / "bin.urdf"),
        }
    except (OSError, ET.ParseError) as exc:
        res.problems.append("could not prepare the descriptions: %r" % (exc,))
        return res
    res.steps.append(BuildStep(
        "urdf_mujoco_extension",
        'inserted <mujoco><compiler fusestatic="false" discardvisual="true" '
        'balanceinertia="true" inertiafromgeom="false"/></mujoco> into all '
        "three descriptions",
        DOC_URDF_EXTENSIONS + "; " + DOC_COMPILER,
        detail="MEASURED on 3.8.1: at the URDF default fusestatic='true' the "
               "block and the bin compile to ZERO named bodies (their geometry "
               "is absorbed into the world body) and the arm loses 'base'. T2 "
               "resolves its roles by name, so the default scene is one the "
               "grader must refuse. inertiafromgeom='false' keeps the declared "
               "masses, per this column's T1 mass audit"))
    res.steps.append(BuildStep(
        "urdf_floating_object",
        "added a URDF link 'world' and a joint 'block_float' of type "
        "'floating' to block.urdf, so the object is a free body",
        DOC_URDF_FLOATING,
        detail="without it the 0.2 kg cube is welded to the world and no arm "
               "can move it; T2.4 would read 'a mass model is attached' and "
               "the object would still be scenery"))
    res.steps.append(BuildStep(
        "urdf_floating_pedestal",
        "added the same floating joint to the ARM's pedestal ('base_float'), "
        "so the 40 kg pedestal rests on the ground rather than being welded "
        "into the world",
        DOC_CONTACT_FILTER,
        detail="MEASURED: with the pedestal welded, MuJoCo's parent-contact "
               "filter stops applying (a welded body joins the WORLD's weld "
               "group, and the filter exempts the world), the pedestal starts "
               "colliding with the shoulder that overlaps it by 10 mm by "
               "design, and joint_1 saturates at its full 150 N.m while moving "
               "0.02 rad in 28 s. The arm simply cannot turn, silently. The "
               "shipped meta.json calls the heavy pedestal the alternative to "
               "a fixed-base convention, so this is the description's own "
               "intent as well as the working fix"))

    # -- step 2: the scene URDF cannot express -----------------------------
    root = mujoco.MjSpec()
    root.modelname = "t2_transfer"
    root.option.timestep = TIMESTEP_S
    root.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    root.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    root.option.impratio = IMPRATIO
    root.option.iterations = SOLVER_ITERATIONS
    root.option.noslip_iterations = NOSLIP_ITERATIONS
    res.steps.append(BuildStep(
        "mjcf_friction_solver",
        "set cone=elliptic, impratio=%g, iterations=%d, noslip_iterations=%d"
        % (IMPRATIO, SOLVER_ITERATIONS, NOSLIP_ITERATIONS),
        DOC_CONE,
        detail="MEASURED on this exact scene: with MuJoCo's defaults "
               "(pyramidal cone, impratio 1, no noslip pass) the block CREEPS "
               "DOWN THROUGH THE JAWS at about 3.5 mm/s -- 42 mm over the 12 s "
               "carry -- under a 30 N squeeze that is nowhere near slipping "
               "physically. That is a T2.3 failure by a factor of two caused "
               "by tangential constraint softness. With these settings the "
               "same carry shows 0.0 mm of creep. NO contact parameter, "
               "geometry, mass or commanded width was changed"))

    ground = root.worldbody.add_body(name="ground")
    gg = ground.add_geom(name="ground_slab")
    gg.type = mujoco.mjtGeom.mjGEOM_BOX
    gg.size = [GROUND_HALF, GROUND_HALF, GROUND_THICK]
    gg.pos = [0.0, 0.0, -GROUND_THICK]
    gg.rgba = [0.35, 0.37, 0.35, 1.0]

    table = root.worldbody.add_body(name="table")
    tg = table.add_geom(name="table_top")
    tg.type = mujoco.mjtGeom.mjGEOM_BOX
    tg.size = [TABLE_HALF[0], TABLE_HALF[1], TABLE_TOP_Z / 2.0]
    tg.pos = [float(radius), 0.0, TABLE_TOP_Z / 2.0]
    tg.rgba = [0.55, 0.45, 0.35, 1.0]

    light = root.worldbody.add_light(name="sun")
    light.pos = [0.0, 0.0, 4.0]
    light.dir = [0.0, 0.0, -1.0]
    light.type = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
    res.steps.append(BuildStep(
        "mjcf_ground_table_light",
        "added static bodies 'ground' (a %g m slab, top at z=0) and 'table' "
        "(top at z=%.2f, centred at radius %.2f), plus one directional light"
        % (2 * GROUND_HALF, TABLE_TOP_Z, radius),
        "URDF has no world, no floor, no furniture and no lighting: those are "
        "scene facts, not robot facts, and no importer of any format could "
        "have supplied them. The ground is a named BOX rather than a plane so "
        "its world-space AABB is exact -- T2.2 measures a lift against the "
        "top of it"))

    lay = layout(radius)
    res.layout = lay
    places = {
        "arm": (0.0, 0.0, 0.0),
        "block": (lay["block_xy_m"][0], lay["block_xy_m"][1],
                  TABLE_TOP_Z + _block_half_height(parts["block"])),
        "bin": (lay["bin_xy_m"][0], lay["bin_xy_m"][1], 0.0),
    }
    for name in ("arm", "block", "bin"):
        try:
            child = mujoco.MjSpec.from_file(str(edited[name]))
        except (ValueError, RuntimeError) as exc:
            res.problems.append("MuJoCo refused %s.urdf: %s" % (name, exc))
            return res
        frame = root.worldbody.add_frame()
        frame.pos = list(places[name])
        try:
            root.attach(child, frame=frame, prefix="")
        except (ValueError, RuntimeError) as exc:
            res.problems.append("could not attach %s: %r" % (name, exc))
            return res
    res.steps.append(BuildStep(
        "mjcf_attach",
        "attached all three descriptions into one scene at frames "
        "(arm %s, block %s, bin %s) with NO name prefix"
        % (places["arm"], tuple(round(v, 4) for v in places["block"]),
           places["bin"]),
        DOC_ATTACH,
        detail="the bodies keep the names the descriptions declare -- 'tool', "
               "'block', 'bin' -- which is what tasks/T2_transfer/meta.json "
               "roles are resolved against"))

    # -- step 3: actuators, which URDF cannot express ----------------------
    efforts = _urdf_efforts(parts["arm"])
    for jn in ARM_JOINTS + FINGER_JOINTS:
        eff = efforts.get(jn)
        try:
            a = root.add_actuator()
            a.name = "pos_%s" % jn
            a.target = jn
            a.trntype = mujoco.mjtTrn.mjTRN_JOINT
            a.gaintype = mujoco.mjtGain.mjGAIN_AFFINE
            a.biastype = mujoco.mjtBias.mjBIAS_AFFINE
            a.gainprm[0] = SERVO_KP[jn]
            a.biasprm[1] = -SERVO_KP[jn]
            a.biasprm[2] = -SERVO_KV[jn]
            if eff:
                a.forcerange = [-float(eff), float(eff)]
                a.forcelimited = 1
            res.actuators.append(a.name)
        except (AttributeError, ValueError, RuntimeError) as exc:
            res.problems.append("could not add an actuator for %r: %r"
                                % (jn, exc))
    res.steps.append(BuildStep(
        "mjcf_actuators",
        "added %d joint-position servos (an affine gain/bias pair, which is "
        "what MJCF's <position> shortcut compiles to), force-limited to the "
        "EFFORT EACH JOINT'S OWN URDF <limit> DECLARES: %s"
        % (len(res.actuators),
           ", ".join("%s=%gN.m" % (k, v) for k, v in sorted(efforts.items()))),
        "URDF cannot express an actuator at all -- <transmission> names a "
        "ros_control interface, not a force -- so a URDF-only scene has nu=0 "
        "and nothing can move the arm. The limits are read out of the shipped "
        "file so no actuator here is stronger than the description allows"))

    try:
        model = root.compile()
    except (ValueError, RuntimeError) as exc:
        res.problems.append("the assembled scene did not compile: %s" % exc)
        return res
    res.bodies = int(model.nbody)
    # The audit is about the THREE DESCRIPTIONS, so the floor and the table
    # this module added are excluded -- they are scene furniture with no
    # declared mass to compare against, and at MuJoCo's 1000 kg/m^3 geom
    # density a 6 m floor slab weighs 3.6 tonnes and would swamp the check.
    scenery = set()
    for nm in ("ground", "table"):
        i = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, nm)
        if i >= 0:
            scenery.add(int(i))
    res.mass_kg_compiled = float(sum(float(m) for i, m in
                                     enumerate(model.body_mass)
                                     if i not in scenery))
    res.scene_mass_kg = float(sum(float(x) for x in model.body_mass))

    missing = [n for n in ("tool", "block", "bin")
               if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n) < 0]
    if missing:
        res.problems.append(
            "the compiled scene carries no body named %s; T2's roles are "
            "resolved by name and the grader will refuse the cell"
            % ", ".join(repr(m) for m in missing))

    declared = sum(v for v in res.mass_kg_declared.values() if v)
    if declared and abs(res.mass_kg_compiled - declared) > 0.02 * declared:
        res.problems.append(
            "the compiled mass (%.3f kg) is more than 2%% off the descriptions' "
            "declared mass (%.3f kg); the scene is not the one the container "
            "describes" % (res.mass_kg_compiled, declared))
    res.steps.append(BuildStep(
        "mass_audit",
        "compiled total mass %.3f kg against the three descriptions' declared "
        "%.3f kg" % (res.mass_kg_compiled, declared),
        "compiler/inertiafromgeom defaults to 'auto' -- infer mass and inertia "
        "from geoms at 1000 kg/m^3 for any link with no <inertial>, which URDF "
        "treats as massless. Set to 'false' in the extension block above"))

    scene = ws / "scene.xml"
    try:
        text, stripped = sanitise_mjcf(root.to_xml())
    except (ValueError, RuntimeError, ET.ParseError) as exc:
        res.problems.append("scene serialisation failed: %r" % (exc,))
        return res
    if stripped:
        res.steps.append(BuildStep(
            "mjcf_strip_empty_defaults",
            "removed %d nameless <default/> elements from the serialised scene"
            % stripped,
            "a MuJoCo 3.8.1 ROUND-TRIP DEFECT, measured here: MjSpec.attach "
            "brings each child's unnamed root <default> along, and to_xml() "
            "writes them out as nested <default/> elements with no class "
            "attribute -- which MuJoCo's OWN parser then rejects with 'XML "
            "Error: empty class name'. The scene compiles in memory and will "
            "not load from disk, so an agent that assembled a scene this way "
            "and handed over the file would ship a deliverable that does not "
            "open. Nothing warns at write time",
            detail="the elements carry no class and no children, so removing "
                   "them cannot change any default the model actually uses; "
                   "the write is verified by loading the file back"))
    try:
        scene.write_text(text, encoding="utf-8")
        res.scene = str(scene)
    except OSError as exc:
        res.problems.append("scene save failed: %r" % (exc,))
        return res

    # The deliverable must stand alone: phase B re-runs the FILE, cold. A
    # scene that only compiles in the memory of the process that built it is
    # not a deliverable, and this is where that gets caught.
    try:
        back = mujoco.MjModel.from_xml_path(str(scene))
    except (ValueError, RuntimeError) as exc:
        res.problems.append("the scene was written but MuJoCo will not load it "
                            "back: %s" % exc)
        return res
    if int(back.nbody) != int(model.nbody):
        res.problems.append(
            "the scene reloads with %d bodies against the %d it compiled with"
            % (back.nbody, model.nbody))
    res.steps.append(BuildStep(
        "reload_check",
        "loaded the written scene back from disk: %d bodies, %d actuators, "
        "%d joints" % (back.nbody, back.nu, back.njnt),
        "phase B re-runs the deliverable COLD from the file "
        "(capability-ladder-plan.md SPEC 2.3), so 'it compiled in memory' is "
        "not evidence that it is a deliverable"))
    return res


def sanitise_mjcf(text):
    """``(text, n_stripped)`` -- MJCF with nameless ``<default/>`` removed.

    See the ``mjcf_strip_empty_defaults`` build step for what this is working
    around. Only childless, attribute-less ``<default>`` elements are touched,
    so nothing a model actually references can be removed.
    """
    root = ET.fromstring(text)
    n = 0
    for parent in root.iter():
        for child in list(parent):
            if child.tag == "default" and not child.attrib and len(child) == 0:
                parent.remove(child)
                n += 1
    if not n:
        return text, 0
    return ET.tostring(root, encoding="unicode"), n


def _block_half_height(block_urdf):
    """Half the object's vertical extent, read off its own description."""
    try:
        root = ET.parse(block_urdf).getroot()
    except (ET.ParseError, OSError):
        return 0.025
    for col in root.iter("collision"):
        box = col.find("./geometry/box")
        if box is not None and box.get("size"):
            try:
                return float(box.get("size").split()[2]) / 2.0
            except (ValueError, IndexError):
                continue
    return 0.025


def _urdf_efforts(arm_urdf):
    """``{joint: effort}`` from the arm description's own ``<limit>`` tags."""
    out = {}
    try:
        root = ET.parse(arm_urdf).getroot()
    except (ET.ParseError, OSError):
        return out
    for j in root.findall("joint"):
        lim = j.find("limit")
        if j.get("name") and lim is not None and lim.get("effort"):
            try:
                out[j.get("name")] = float(lim.get("effort"))
            except ValueError:
                continue
    return out


def write_build_record(res, path):
    Path(path).write_text(json.dumps(res.as_dict(), indent=2),
                          encoding="utf-8")


__all__ = ["ARM_JOINTS", "FINGER_JOINTS", "GROUND_HALF", "IMPRATIO",
           "NOSLIP_ITERATIONS", "SOLVER_ITERATIONS", "T2BuildResult",
           "TABLE_TOP_Z", "TIMESTEP_S", "WORK_RADIUS", "build_t2_scene",
           "layout", "prepare_urdf", "sanitise_mjcf", "write_build_record"]
