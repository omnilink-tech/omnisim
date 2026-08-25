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

"""**One shipped two-legged description -> a MuJoCo scene it can walk in.**

T4's container ships *"one two-legged robot description plus the one requirement
the world has to satisfy, and nothing else, on every column"*
(``tasks/T4_humanoid/meta.json``): no ground, no world, no light, no placement,
no actuator, no controller, no gait -- **and no support rig**, because the tier
permits one and measures it, and providing one would hand every column a
technique this project happens to rely on. Bringing that robot into a scene is
part of the capability under test, so this module is the honest record of what
it costs on MuJoCo -- every edit a cited
:class:`~ladder.adapters.mujoco.model_build.BuildStep`, exactly as the T1, T2
and T3 builders do.

**It is a scripted fixture, not a ladder cell.** A cell is an autonomous agent
given one sentence (``capability-ladder-plan.md`` §2). What this proves is that
the task is *achievable*, which ``meta.json`` ->
``container.authored_here.before_the_freeze`` makes a precondition of the
freeze.

What is the same as one rung down, and what is not
--------------------------------------------------

**Same, and it is the single most likely blocker on any column:** ⚠ **URDF
cannot express rotor inertia, and this robot needs it.** ``<dynamics>`` carries
damping and friction and nothing else, so ``armature`` defaults to **0** and a
stiff position servo on a 1.8 kg shank at a 2 ms step is numerically unstable.
:data:`ARMATURE` and :data:`DAMPING` are set on the MJCF side. The container's
own ``PROVENANCE.txt`` says this out loud and adds the sentence that matters
here: *"a two-legged robot has less margin than a four-legged one, not more"*.

**Same:** the root link has no joint to the world, ``compiler/fusestatic``
defaults to **true for URDF**, and a jointless root is not merely welded but
**absorbed into the world body** -- so the compiled scene carries no
``base_link`` at all and T4, which grades the base by name, must refuse it.
Fixed with ``fusestatic="false"`` plus URDF's own ``<joint type="floating">``.

**Not the same: the floor has to be long enough for the task's own clock.**
``meta.json`` -> ``phases.standalone.duration_s`` is **300 s**, four and a half
times the arithmetic minimum, so that a walker *slower* than the speed floor is
measured as slow rather than truncated. A gait that makes 0.27 m/s covers about
**80 m** in that window, so a T3-sized 45 m slab would end this run by the robot
**walking off the edge** -- which the fall test reads as a fall, at a
z the arena channel cannot explain away. :data:`GROUND_X` is therefore 150 m
long. That is not generosity: it is the task's own duration multiplied by the
gait's own speed, and it is what makes ``termination_cause`` a statement about
the walk rather than about the slab.

**Not the same: this scene can be built with a rig, and the rig is the point.**
``rig="weld"`` adds a mocap body and a ``<weld>`` equality constraint binding
the base to it. Nothing about it is a recommendation -- it exists to
**demonstrate the hole** ``meta.json`` ->
``container.authored_here.an_open_question_the_demonstration_exposed`` records:
a rig built as a constraint holds a robot just as firmly as one built as an
applied wrench and totals **zero** on ``xfrc_applied``. See
:mod:`ladder.adapters.mujoco.runner_t4`, which counts the constraint reaction
so that this column cannot publish a held robot as ``T4-unsupported``, and
``BRINGUP_T4.md`` §6 for what the grader said before and after.
"""

from __future__ import annotations

import json
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from ladder.adapters.mujoco.model_build import (  # noqa: E402
    BuildStep, DOC_COMPILER, DOC_URDF_EXTENSIONS, urdf_declared_mass)
from ladder.adapters.mujoco.t2_scene import sanitise_mjcf  # noqa: E402
from ladder.adapters.mujoco.t3_scene import (  # noqa: E402
    DOC_ARMATURE, DOC_FRICTION, DOC_INTEGRATOR, DOC_URDF_FLOATING,
    urdf_efforts)

# --- citations ---------------------------------------------------------------

DOC_WELD = (
    "MuJoCo XML reference, equality/weld: 'This element creates a weld "
    "equality constraint. It attaches two bodies to each other, removing all "
    "relative degrees of freedom between them.' Combined with body/mocap "
    "('the body is a mocap body ... its position and orientation are set "
    "directly by the user'), it is the standard way to drive or hold a "
    "floating base -- and it applies NOTHING through mjData.xfrc_applied")
DOC_EFC_ORDER = (
    "MuJoCo Computation chapter, 'Constraint solver': the rows of the "
    "constraint Jacobian are assembled in the order equality, friction loss, "
    "joint/tendon limit, contact, and mjData.ne / mjData.nf record the first "
    "two counts -- which is what makes the EQUALITY reaction separable from "
    "the contact reaction the tier excludes")

# --- the scene, in metres ----------------------------------------------------

TIMESTEP_S = 0.002
ARMATURE = 0.02             # kg.m^2 on every hinge. URDF has no field for it.
DAMPING = 0.6               # N.m.s/rad, on top of the URDF's own 0.10
SERVO_KP = 400.0
SERVO_KV = 25.0

GROUND_FRICTION = (1.2, 0.005, 0.0001)
GROUND_THICK = 0.10
# The walking region. Long in +x because that is the direction the gait walks.
# The length is DERIVED, not chosen: the task's own standalone window is 300 s
# and this gait makes about 0.27 m/s, so a floor shorter than ~85 m ends the run
# by the robot walking off its edge -- which the fall test reads as a fall.
# 150 m is that number with room to spare, against a task requirement of 15 m.
GROUND_X = (-5.0, 145.0)
GROUND_Y = (-6.0, 6.0)
GROUND_PLANE_HALF = 200.0   # only when ground="plane"

BASE_LINK = "base_link"
GROUND_BODY = "ground"
RIG_ANCHOR = "rig_anchor"   # only when rig="weld"
RIG_WELD = "rig_weld"

LEGS = ("l", "r")
JOINTS = tuple("%s_%s_joint" % (pre, leg) for leg in LEGS
               for pre in ("hip_yaw", "hip_roll", "hip_pitch", "knee",
                           "ankle_pitch", "ankle_roll"))

# The commanded hip-pitch-axis-to-sole distance the gait stands at, and the
# height the base is therefore dropped from. Both are solved from the shipped
# description's own geometry rather than typed: the hip pitch axis sits
# 0.03 + 0.055 + 0.02 = 0.105 m below the base origin, so a base at
# 0.105 + 0.50 = 0.605 m puts the sole exactly on the floor. The extra 2 mm is
# settle clearance, not a drop test.
STAND_HEIGHT_M = 0.50
HIP_PITCH_BELOW_BASE_M = 0.105
SPAWN_Z = HIP_PITCH_BELOW_BASE_M + STAND_HEIGHT_M + 0.002

_MUJOCO_BLOCK = (
    '  <!-- Added by the capability-ladder MuJoCo column (T4). fusestatic is\n'
    '       the load-bearing one: at its URDF default of "true" a jointless\n'
    '       root link is ABSORBED INTO THE WORLD BODY, and this robot then has\n'
    '       no base_link at all. T4 grades the base BY NAME, so that scene is\n'
    '       ungradeable. inertiafromgeom="false" keeps the declared masses. -->\n'
    '  <mujoco>\n'
    '    <compiler strippath="true" fusestatic="false" discardvisual="true"\n'
    '              balanceinertia="true" inertiafromgeom="false"\n'
    '              angle="radian"/>\n'
    '  </mujoco>\n')

_FLOAT_JOINT = (
    '  <!-- Added by the capability-ladder MuJoCo column (T4): URDF\'s own\n'
    '       floating joint, so %(base)s is a free body standing on the ground\n'
    '       rather than scenery welded to the world. -->\n'
    '  <link name="world"/>\n'
    '  <joint name="%(base)s_float" type="floating">\n'
    '    <parent link="world"/>\n'
    '    <child link="%(base)s"/>\n'
    '    <origin rpy="0 0 0" xyz="0 0 %(z)s"/>\n'
    '  </joint>\n')


@dataclass
class T4BuildResult:
    """Everything the T4 scene build produced, and everything it changed."""

    workspace: str = ""
    scene: str = ""
    urdf_source: str = ""
    urdf_edited: str = ""
    robot_name: str = ""
    base_link: str = BASE_LINK
    ground: dict = field(default_factory=dict)
    rig: dict = field(default_factory=dict)
    joints: list = field(default_factory=list)
    efforts: dict = field(default_factory=dict)
    force_limited: bool = False
    mass_kg_declared: float = None
    mass_kg_compiled: float = None       # the robot's bodies only
    scene_mass_kg: float = None          # including the slab
    bodies: int = None
    actuators: list = field(default_factory=list)
    steps: list = field(default_factory=list)
    problems: list = field(default_factory=list)

    def as_dict(self):
        return {"workspace": self.workspace, "scene": self.scene,
                "urdf_source": self.urdf_source,
                "urdf_edited": self.urdf_edited,
                "robot_name": self.robot_name, "base_link": self.base_link,
                "ground": dict(self.ground), "support_rig": dict(self.rig),
                "joints": list(self.joints),
                "urdf_declared_effort_nm": dict(self.efforts),
                "actuators_force_limited_to_the_urdf_effort": self.force_limited,
                "mass_kg_declared_by_urdf": self.mass_kg_declared,
                "mass_kg_in_the_compiled_model": self.mass_kg_compiled,
                "scene_mass_kg_including_the_ground": self.scene_mass_kg,
                "bodies": self.bodies, "actuators": list(self.actuators),
                "steps": [s.as_dict() for s in self.steps],
                "problems": list(self.problems)}


def prepare_urdf(src, dest, *, floating_base=BASE_LINK, spawn_z=SPAWN_Z):
    """Copy the shipped description and insert MuJoCo's own extension block.

    Never edits the container in place: the description is the fixture every
    later run reads. The edited file is re-parsed with a **strict** XML reader
    afterwards, because MuJoCo's URDF path is explicitly not schema-checked and
    a file only one of the two readers accepts is a file whose provenance
    nobody can check.
    """
    text = Path(src).read_text(encoding="utf-8")
    add = _MUJOCO_BLOCK
    if floating_base:
        add += _FLOAT_JOINT % {"base": floating_base, "z": "%g" % spawn_z}
    out = re.sub(r"(<robot\b[^>]*>)", lambda m: m.group(1) + "\n" + add,
                 text, count=1)
    Path(dest).write_text(out, encoding="utf-8")
    ET.fromstring(out)
    return Path(dest)


def build_t4_scene(container_dir, workspace, *, ground="box", rig="wrench",
                   armature=ARMATURE, damping=DAMPING, kp=SERVO_KP,
                   kv=SERVO_KV, spawn_z=SPAWN_Z, force_limits=True):
    """One shipped description in, one driveable MJCF scene out. Never raises.

    ``container_dir`` is ``tasks/T4_humanoid/container``. Returns a
    :class:`T4BuildResult`; anything that went wrong is in ``problems`` and the
    scene path is left empty.

    ``rig`` selects what, if anything, the scene provides for the *support*
    half of the tier:

    ``"wrench"``
        nothing at all on the scene side. The driver applies its rig through
        ``mjData.xfrc_applied``, which is the channel T4.4 measures. This is
        the recorded recipe and the default.
    ``"none"``
        also nothing, and the driver applies nothing either -- the same script
        with the wrench switched off, which is the tier's ``T4-unsupported``
        attempt.
    ``"weld"``
        a mocap body and a ``<weld>`` equality binding the base to it. This is
        **not** a recommended technique: it is the executable form of the open
        question the task file records, and it exists so the claim *"a
        constraint rig totals zero applied wrench"* can be tested rather than
        argued about.

    ``force_limits`` clamps every actuator to the effort the URDF declares and
    **defaults to on**, which is a deliberate divergence from the recorded
    recipe (it stated "no force limit"). The runner measures the peak force per
    joint against the declared efforts either way, so the comparison stays a
    measurement rather than an assumption.
    """
    import mujoco

    res = T4BuildResult(workspace=str(workspace),
                        force_limited=bool(force_limits))
    src = Path(container_dir)
    ws = Path(workspace)
    ws.mkdir(parents=True, exist_ok=True)
    pkg = ws / "container"
    if pkg.exists():
        shutil.rmtree(pkg)
    shutil.copytree(src, pkg)
    res.steps.append(BuildStep(
        "copy", "copied the shipped robot description into the run workspace",
        "the ladder ships the task as files; editing them in place would "
        "mutate the fixture every later run reads", detail=str(pkg)))

    cands = sorted(pkg.rglob("*.urdf"))
    if not cands:
        res.problems.append("the container has no .urdf under %s" % pkg)
        return res
    urdf = cands[0]
    res.urdf_source = str(src / urdf.relative_to(pkg))
    res.urdf_edited = str(ws / "strider.urdf")
    res.mass_kg_declared = urdf_declared_mass(urdf)
    res.efforts = urdf_efforts(urdf)
    try:
        res.robot_name = ET.parse(urdf).getroot().get("name") or ""
    except (ET.ParseError, OSError):
        res.robot_name = ""

    # -- step 1: MuJoCo's URDF extension block + the floating base ---------
    try:
        edited = prepare_urdf(urdf, res.urdf_edited, spawn_z=spawn_z)
    except (OSError, ET.ParseError) as exc:
        res.problems.append("could not prepare the description: %r" % (exc,))
        return res
    res.steps.append(BuildStep(
        "urdf_mujoco_extension",
        'inserted <mujoco><compiler strippath="true" fusestatic="false" '
        'discardvisual="true" balanceinertia="true" inertiafromgeom="false" '
        'angle="radian"/></mujoco> into the description',
        DOC_URDF_EXTENSIONS + "; " + DOC_COMPILER,
        detail="at the URDF default fusestatic='true' a jointless root link is "
               "ABSORBED INTO THE WORLD BODY and the compiled scene carries no "
               "body named 'base_link'. T4 grades the base by name "
               "(meta.json -> robot.declared_name), so the grader would refuse "
               "the cell. inertiafromgeom='false' keeps the declared masses. "
               "discardvisual='true' drops the visual-only geoms and CANNOT "
               "change the physics here: this description's collision set is "
               "the base box and the two foot boxes, and every other geom has "
               "contype=conaffinity=0"))
    res.steps.append(BuildStep(
        "urdf_floating_base",
        "added a URDF link 'world' and a joint '%s_float' of type 'floating' "
        "at z = %g m, so the robot is a free body standing on the ground"
        % (BASE_LINK, spawn_z),
        DOC_URDF_FLOATING,
        detail="the spawn height is the commanded stance solved backwards off "
               "the description's own geometry: the hip pitch axis sits "
               "0.03 + 0.055 + 0.02 = %g m below the base origin and the "
               "commanded hip-to-sole distance is %g m, so a base at %g m puts "
               "the sole exactly on the floor. The extra 2 mm is settle "
               "clearance, not a drop test"
               % (HIP_PITCH_BELOW_BASE_M, STAND_HEIGHT_M,
                  HIP_PITCH_BELOW_BASE_M + STAND_HEIGHT_M)))

    try:
        spec = mujoco.MjSpec.from_file(str(edited))
    except (ValueError, RuntimeError) as exc:
        res.problems.append("MuJoCo refused the description: %s" % exc)
        return res

    # -- step 2: the integration this robot needs, which URDF cannot say ---
    spec.option.timestep = float(TIMESTEP_S)
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    hinges = 0
    for j in spec.joints:
        if int(j.type) == int(mujoco.mjtJoint.mjJNT_HINGE):
            j.armature = float(armature)
            # MjsJoint.damping is an mjtNum[3] (ball and free joints have
            # three), so a hinge's single value is written into all three
            # slots; the compiler reads only the first for a hinge.
            j.damping = [float(damping)] * 3
            hinges += 1
            res.joints.append(j.name)
    res.steps.append(BuildStep(
        "mjcf_armature_and_integrator",
        "set joint/armature = %g kg.m^2 and joint/damping = %g N.m.s/rad on "
        "all %d hinges and option/integrator = implicitfast at a %g s timestep"
        % (armature, damping, hinges, TIMESTEP_S),
        DOC_ARMATURE + "; " + DOC_INTEGRATOR,
        detail="THE SINGLE MOST LIKELY FIRST BLOCKER ON ANY COLUMN, and the "
               "container's own PROVENANCE.txt states it: URDF has NO field "
               "for rotor inertia -- <dynamics> carries damping and friction "
               "and nothing else -- so the importer default is 0, and a kp = "
               "%g position servo on a 1.8 kg shank at a %g s step is "
               "numerically unstable. The failure reads exactly like a physics "
               "or modelling defect and nothing warns. The damping is on TOP "
               "of the 0.10 the description declares per joint"
               % (kp, TIMESTEP_S)))

    # -- step 3: the ground, which URDF cannot express ---------------------
    _add_ground(mujoco, spec, ground, res)

    # -- step 4: actuators, which URDF cannot express either ---------------
    _add_actuators(mujoco, spec, kp, kv, res, force_limits=force_limits)

    # -- step 4b: the support rig, if this build is making one -------------
    _add_rig(mujoco, spec, rig, res, spawn_z=spawn_z)

    try:
        model = spec.compile()
    except (ValueError, RuntimeError) as exc:
        res.problems.append("the assembled scene did not compile: %s" % exc)
        return res
    res.bodies = int(model.nbody)
    res.scene_mass_kg = float(sum(float(x) for x in model.body_mass))
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, GROUND_BODY)
    res.mass_kg_compiled = float(
        sum(float(m) for i, m in enumerate(model.body_mass) if i != gid))

    if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, BASE_LINK) < 0:
        res.problems.append(
            "the compiled scene carries no body named %r; T4 grades the base "
            "by name and the grader will refuse the cell" % BASE_LINK)
    d = res.mass_kg_declared
    if d and abs(res.mass_kg_compiled - d) > 0.02 * d:
        res.problems.append(
            "the compiled robot mass (%.3f kg) is more than 2%% off the "
            "description's declared mass (%.3f kg); the robot being simulated "
            "is not the robot the description declares"
            % (res.mass_kg_compiled, d))
    res.steps.append(BuildStep(
        "mass_audit",
        "compiled robot mass %.3f kg against the description's declared "
        "%.3f kg" % (res.mass_kg_compiled, d or float("nan")),
        "compiler/inertiafromgeom defaults to 'auto' -- infer mass and inertia "
        "from geoms at 1000 kg/m^3 for any link with no <inertial>, which URDF "
        "treats as massless. Set to 'false' in the extension block above",
        detail="the cell boundary of this tier is 0.02 x m.g and the published "
               "figure is a multiple of body weight, so a wrong mass is a "
               "wrong CELL and not merely a wrong annotation"))

    # -- step 5: write it, strip the round-trip defect, load it back -------
    scene = ws / "scene.xml"
    try:
        text, stripped = sanitise_mjcf(spec.to_xml())
    except (ValueError, RuntimeError, ET.ParseError) as exc:
        res.problems.append("scene serialisation failed: %r" % (exc,))
        return res
    if stripped:
        res.steps.append(BuildStep(
            "mjcf_strip_empty_defaults",
            "removed %d nameless <default/> elements from the serialised scene"
            % stripped,
            "a MuJoCo 3.8.1 round-trip defect, measured during this column's "
            "T2 bring-up: to_xml() can write <default/> elements with no class "
            "attribute, which MuJoCo's OWN parser then rejects with 'XML "
            "Error: empty class name'. The scene compiles in memory and will "
            "not load from disk"))
    try:
        scene.write_text(text, encoding="utf-8")
        res.scene = str(scene)
    except OSError as exc:
        res.problems.append("scene save failed: %r" % (exc,))
        return res

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
    if float(back.dof_armature.max()) <= 0.0:
        res.problems.append(
            "the reloaded scene has zero armature on every dof; the standing "
            "pose will not hold (see the mjcf_armature_and_integrator step)")
    if int(back.neq) != int(model.neq):
        res.problems.append(
            "the scene reloads with %d equality constraints against the %d it "
            "compiled with -- and an equality constraint on the base is a "
            "support rig the wrench channel cannot see"
            % (back.neq, model.neq))
    res.steps.append(BuildStep(
        "reload_check",
        "loaded the written scene back from disk: %d bodies, %d actuators, "
        "%d joints, %d equality constraints, peak dof armature %g kg.m^2"
        % (back.nbody, back.nu, back.njnt, back.neq,
           float(back.dof_armature.max())),
        "phase B re-runs the deliverable COLD from the file "
        "(capability-ladder-plan.md SPEC 2.3), so 'it compiled in memory' is "
        "not evidence that it is a deliverable -- and the armature is checked "
        "on the RELOADED model because it is the one setting here that no "
        "format the robot arrived in could carry"))
    return res


def _add_ground(mujoco, spec, kind, res):
    """A named static body carrying the floor, plus one light.

    Named rather than a bare world geom because T4.4 asserts that *nothing but
    the ground* touched the robot, and ``world`` as a contact partner tells a
    reader nothing about what the robot was standing on.
    """
    try:
        body = spec.worldbody.add_body()
        body.name = GROUND_BODY
        g = body.add_geom()
        g.name = "ground_surface"
        g.friction = list(GROUND_FRICTION)
        g.rgba = [0.35, 0.37, 0.35, 1.0]
        if kind == "plane":
            g.type = mujoco.mjtGeom.mjGEOM_PLANE
            g.size = [GROUND_PLANE_HALF, GROUND_PLANE_HALF, 0.5]
            g.pos = [0.0, 0.0, 0.0]
            res.ground = {"kind": "plane",
                          "declared_half_size_m": GROUND_PLANE_HALF,
                          "bounded": False, "friction": list(GROUND_FRICTION)}
            what = ("added a static body 'ground' carrying an INFINITE plane "
                    "(declared half-size %g m) and one directional light"
                    % GROUND_PLANE_HALF)
            why = ("URDF has no world, no floor and no lighting: those are "
                   "scene facts, not robot facts. This is the recorded "
                   "recipe's own floor. A plane's world AABB is +/-1e10 m, so "
                   "the walking region derived from it is true, unreadable, "
                   "and useless to the arena channel this tier makes a "
                   "completeness requirement")
        else:
            g.type = mujoco.mjtGeom.mjGEOM_BOX
            hx = 0.5 * (GROUND_X[1] - GROUND_X[0])
            hy = 0.5 * (GROUND_Y[1] - GROUND_Y[0])
            g.size = [hx, hy, GROUND_THICK / 2.0]
            g.pos = [0.5 * (GROUND_X[0] + GROUND_X[1]),
                     0.5 * (GROUND_Y[0] + GROUND_Y[1]),
                     -GROUND_THICK / 2.0]
            res.ground = {"kind": "box", "x_m": list(GROUND_X),
                          "y_m": list(GROUND_Y), "top_z_m": 0.0,
                          "bounded": True, "friction": list(GROUND_FRICTION)}
            what = ("added a static body 'ground' carrying a %g x %g m box "
                    "slab with its top at z = 0, and one directional light"
                    % (GROUND_X[1] - GROUND_X[0], GROUND_Y[1] - GROUND_Y[0]))
            why = ("URDF has no world, no floor and no lighting. The floor is "
                   "a BOX rather than a plane so its world-space AABB is exact "
                   "-- that box IS the walking region the arena channel "
                   "reports, and on THIS tier a run whose recorder cannot see "
                   "the edge of the world is an INCOMPLETE cell rather than a "
                   "quiet one. Its LENGTH is derived: the task's own "
                   "standalone window is 300 s and this gait makes about "
                   "0.27 m/s, so a shorter floor would end the run by the "
                   "robot walking off the edge -- which the fall test reads as "
                   "a fall. The task requires 15 m; this offers %g"
                   % (GROUND_X[1] - 0.0))
        light = spec.worldbody.add_light()
        light.name = "sun"
        light.pos = [0.0, 0.0, 6.0]
        light.dir = [0.0, 0.0, -1.0]
        light.type = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
        res.steps.append(BuildStep("mjcf_ground_and_light", what, why,
                                   detail=DOC_FRICTION + "; friction %s"
                                          % (list(GROUND_FRICTION),)))
    except (AttributeError, ValueError, RuntimeError) as exc:
        res.problems.append("could not add the ground: %r" % (exc,))


def _add_actuators(mujoco, spec, kp, kv, res, *, force_limits=False):
    """One affine joint-position servo per hinge.

    MJCF's ``<position>`` shortcut compiles to an affine gain/bias pair and the
    spec API has no shortcut elements, so the pair is set directly. Same
    actuator either way -- the saved MJCF is what a reader can check.
    """
    for jn in JOINTS:
        try:
            a = spec.add_actuator()
            a.name = "pos_%s" % jn
            a.target = jn
            a.trntype = mujoco.mjtTrn.mjTRN_JOINT
            a.gaintype = mujoco.mjtGain.mjGAIN_AFFINE
            a.biastype = mujoco.mjtBias.mjBIAS_AFFINE
            a.gainprm[0] = float(kp)
            a.biasprm[1] = -float(kp)
            a.biasprm[2] = -float(kv)
            eff = res.efforts.get(jn)
            if force_limits and eff:
                a.forcerange = [-float(eff), float(eff)]
                a.forcelimited = 1
            res.actuators.append(a.name)
        except (AttributeError, ValueError, RuntimeError) as exc:
            res.problems.append("could not add an actuator for %r: %r"
                                % (jn, exc))
    res.steps.append(BuildStep(
        "mjcf_actuators",
        "added %d joint-position servos, kp = %g, kv = %g, force limits %s"
        % (len(res.actuators), kp, kv,
           ("clamped to each joint's own URDF <limit effort>"
            if force_limits else "OFF -- the recorded recipe's setting")),
        "URDF cannot express an actuator at all -- <transmission> names a "
        "ros_control interface, not a force -- so a URDF-only scene has nu=0 "
        "and nothing can move the robot",
        detail="the efforts the description declares are %s. Every actuator is "
               "an mjTRN_JOINT transmission on a HINGE and none of them is on "
               "the base's free joint, which is half of what makes the "
               "applied-wrench total COMPLETE rather than partial (the other "
               "half is the equality-constraint count). The runner MEASURES "
               "the peak actuator force per joint against these efforts either "
               "way"
               % ", ".join("%s=%gN.m" % (k, v)
                           for k, v in sorted(res.efforts.items()))))


def _add_rig(mujoco, spec, rig, res, *, spawn_z=SPAWN_Z):
    """The scene-side half of a support rig. Only ``rig="weld"`` builds one.

    ⚠ **This is a demonstration of a hole in the tier, not a recommended
    technique.** ``meta.json`` ->
    ``container.authored_here.an_open_question_the_demonstration_exposed``:
    *"A rig implemented instead as a weld, an equality constraint, a mocap
    attachment or a kinematic base would hold the robot just as firmly and the
    wrench channel would read ZERO -- and the run would be published in
    T4-unsupported, which is the cell the plan says must be 'numerically
    nothing'."* This builds exactly that rig so the sentence can be tested.
    """
    if rig != "weld":
        res.rig = {"kind": rig,
                   "scene_side": "nothing -- the rig, if any, is the driver's "
                                 "own applied wrench through mjData."
                                 "xfrc_applied, which is the channel this tier "
                                 "measures"}
        return
    try:
        anchor = spec.worldbody.add_body()
        anchor.name = RIG_ANCHOR
        anchor.mocap = True
        anchor.pos = [0.0, 0.0, float(spawn_z)]
        eq = spec.add_equality()
        eq.name = RIG_WELD
        eq.type = mujoco.mjtEq.mjEQ_WELD
        eq.objtype = mujoco.mjtObj.mjOBJ_BODY
        eq.name1 = BASE_LINK
        eq.name2 = RIG_ANCHOR
        eq.active = True
        res.rig = {"kind": "weld", "anchor_body": RIG_ANCHOR,
                   "equality": RIG_WELD, "welded_to": BASE_LINK,
                   "scene_side": "a mocap body and a weld equality binding the "
                                 "base to it. NOTHING reaches mjData."
                                 "xfrc_applied: the whole reaction is a "
                                 "constraint force, which is the point"}
        res.steps.append(BuildStep(
            "mjcf_constraint_support_rig",
            "added a mocap body %r and a weld equality %r binding %r to it"
            % (RIG_ANCHOR, RIG_WELD, BASE_LINK),
            DOC_WELD,
            detail="⚠ BUILT ON PURPOSE TO BE MEASURED, NOT RECOMMENDED. It "
                   "holds the robot exactly as firmly as an applied wrench "
                   "would and contributes ZERO to mjData.xfrc_applied, so a "
                   "column that attests only that array publishes a held robot "
                   "in the cell reserved for numerically nothing. "
                   "runner_t4 counts the equality reaction (%s) so that this "
                   "column cannot" % DOC_EFC_ORDER))
    except (AttributeError, ValueError, RuntimeError) as exc:
        res.problems.append("could not add the weld rig: %r" % (exc,))


def write_build_record(res, path):
    Path(path).write_text(json.dumps(res.as_dict(), indent=2),
                          encoding="utf-8")


__all__ = ["ARMATURE", "BASE_LINK", "DAMPING", "DOC_EFC_ORDER", "DOC_WELD",
           "GROUND_BODY", "GROUND_FRICTION", "GROUND_PLANE_HALF",
           "GROUND_THICK", "GROUND_X", "GROUND_Y", "HIP_PITCH_BELOW_BASE_M",
           "JOINTS", "LEGS", "RIG_ANCHOR", "RIG_WELD", "SERVO_KP", "SERVO_KV",
           "SPAWN_Z", "STAND_HEIGHT_M", "TIMESTEP_S", "T4BuildResult",
           "build_t4_scene", "prepare_urdf", "urdf_efforts",
           "write_build_record"]
