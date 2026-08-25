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

"""**One shipped quadruped description -> a MuJoCo scene it can walk in.**

T3's container ships *"one four-legged robot description, and nothing else, on
every column"* (``tasks/T3_quadruped/meta.json``): no ground, no world, no
light, no placement, no actuator, no controller and no gait. Bringing that
robot into a scene is part of the capability under test, so this module is the
honest record of what it costs on MuJoCo -- every edit a cited
:class:`~ladder.adapters.mujoco.model_build.BuildStep`, exactly as the T1 and
T2 builders do.

**It is a scripted fixture, not a ladder cell.** A cell is an autonomous agent
given one sentence (``capability-ladder-plan.md`` §2). What this proves is that
the task is *achievable* -- which ``meta.json`` ->
``container.authored_here.before_the_freeze`` makes a precondition of the
freeze, because a robot no column can walk makes the cell a measurement of our
asset rather than of anybody's agent.

Two things bite, and the first one is the single most likely blocker anywhere
--------------------------------------------------------------------------

**1. ⚠ URDF CANNOT EXPRESS ROTOR INERTIA, AND WITHOUT IT THIS ROBOT CANNOT
STAND.** It is a numerical failure that reads exactly like a physics or
modelling defect. A position servo at ``kp = 250`` on a shank whose own inertia
is ``8.5e-4 kg.m^2`` has an undamped period shorter than two 2 ms steps, so with
``armature`` at the importer's default of **0** and an explicit integrator the
robot *bounces across the floor while being told to hold a fixed standing
pose*: measured, about 1 m/s of drift, over on its back inside 3 s, peak joint
tracking error 0.42 rad. With :data:`ARMATURE` on every hinge and an implicit
integrator, the same command holds the base still for 20 s with all four feet
down. URDF has **no field for it** -- ``<dynamics>`` carries damping and
friction and nothing else -- so no importer of any format could have supplied
it, and nothing warns. Lowering ``kp`` instead also works (``kp = 60`` is stable
at the same step) and is the other legitimate fix.

**2. The root link has no joint to the world, so an importer welds it down.**
T1's finding, unchanged, and it applies to every URDF: MuJoCo's ``fusestatic``
defaults to **true for URDF**, which does not merely weld a jointless root but
*absorbs it into the world body*, and the robot then has no ``base_link`` at
all. T3 grades the base **by name** (``meta.json`` -> ``robot.declared_name``),
so a scene built on the defaults is one the grader must refuse. Fixed with
``fusestatic="false"`` plus URDF's own ``<joint type="floating">``.

The ground is a **finite named slab**, and that is a deliberate divergence
------------------------------------------------------------------------

The scratch harness that first demonstrated this walk used an *infinite plane*.
This module ships a finite box slab instead, for a reason that belongs to T3
specifically: **``ArenaBounds`` is a graded-adjacent channel and an infinite
plane cannot answer it.** MuJoCo gives a plane geom an AABB of ``±1e10`` m, so a
column that derived the walking region from it would print *"the longest
straight run the region allowed: 14142135623 m"* in every row -- which is true,
useless, and hides the one thing the arena channel exists for
(``docs/developer/g1-endurance-2026-08-01.md`` §8: six clean runs of this
tree's flagship were ended by a wall and would have been published as a
degrading gait). A bounded floor is also strictly *harder* than an unbounded
one, so the divergence cannot flatter the run. :func:`build_t3_scene` keeps the
plane available (``ground="plane"``) precisely so the two can be compared, and
``BRINGUP_T3.md`` §4.3 reports that comparison rather than asserting it.
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

# --- citations ---------------------------------------------------------------

DOC_ARMATURE = (
    "MuJoCo XML reference, joint/armature "
    "(https://mujoco.readthedocs.io/en/stable/XMLreference.html): 'Armature "
    "inertia (or rotor inertia, or reflected inertia) of all degrees of "
    "freedom created by this joint. These are constants added to the "
    "diagonal of the inertia matrix in generalized coordinates. They make the "
    "simulation more stable, and often increase physical realism.' URDF has no "
    "field for it at all -- <dynamics> carries damping and friction and "
    "nothing else -- so the importer cannot supply it and the default is 0")
DOC_INTEGRATOR = (
    "MuJoCo Computation chapter, 'Numerical integration', and the XML "
    "reference for option/integrator: implicitfast integrates the velocity-"
    "dependent forces implicitly, which is what makes a stiff position servo "
    "stable at a step size an explicit integrator cannot hold")
DOC_URDF_FLOATING = (
    "URDF's own joint type='floating', honoured by MuJoCo's URDF parser "
    "(verified on 3.8.1 in this column's T1 bring-up, BRINGUP.md 3.2 stage 11). "
    "A URDF root link has no joint to the world and MuJoCo welds a jointless "
    "body to its parent")
DOC_FRICTION = (
    "MuJoCo XML reference, geom/friction: the triple is sliding, torsional and "
    "rolling friction. A legged robot standing on four spheres is decided by "
    "the first of them; the two small ones stop a sphere behaving like a "
    "ball bearing")

# --- the scene, in metres ----------------------------------------------------

TIMESTEP_S = 0.002
ARMATURE = 0.008            # kg.m^2 on every hinge. See finding 1.
SERVO_KP = 250.0
SERVO_KV = 10.0

GROUND_FRICTION = (1.2, 0.005, 0.0001)
GROUND_THICK = 0.10
# The walking region. Long in +x because that is the direction the gait walks,
# and wide enough that a drifting crawl does not fall off the side. The task
# states a minimum free run-up of 15.0 m (MIN_RUN_UP_FACTOR x the 10 m bar);
# this offers 45 m of straight run from the origin.
GROUND_X = (-5.0, 45.0)
GROUND_Y = (-6.0, 6.0)
GROUND_PLANE_HALF = 60.0    # only when ground="plane"

# Where the base is dropped. The commanded stance puts each foot centre
# STAND_HEIGHT below its hip, the hips sit 0.02 m below the base origin, and
# the foot spheres have radius 0.022 -- so the resting base is at 0.282 m and
# this is three millimetres of settle, not a drop test.
SPAWN_Z = 0.285

BASE_LINK = "base_link"
GROUND_BODY = "ground"
LEGS = ("fl", "fr", "rl", "rr")
JOINTS = tuple("%s_%s_joint" % (pre, leg)
               for leg in LEGS for pre in ("hip", "thigh", "knee"))

_MUJOCO_BLOCK = (
    '  <!-- Added by the capability-ladder MuJoCo column (T3). fusestatic is\n'
    '       the load-bearing one: at its URDF default of "true" a jointless\n'
    '       root link is ABSORBED INTO THE WORLD BODY, and this robot then has\n'
    '       no base_link at all. T3 grades the base BY NAME, so that scene is\n'
    '       ungradeable. inertiafromgeom="false" keeps the declared masses. -->\n'
    '  <mujoco>\n'
    '    <compiler strippath="true" fusestatic="false" discardvisual="true"\n'
    '              balanceinertia="true" inertiafromgeom="false"/>\n'
    '  </mujoco>\n')

_FLOAT_JOINT = (
    '  <!-- Added by the capability-ladder MuJoCo column (T3): URDF\'s own\n'
    '       floating joint, so %(base)s is a free body rather than scenery. -->\n'
    '  <link name="world"/>\n'
    '  <joint name="%(base)s_float" type="floating">\n'
    '    <parent link="world"/>\n'
    '    <child link="%(base)s"/>\n'
    '    <origin rpy="0 0 0" xyz="0 0 %(z)s"/>\n'
    '  </joint>\n')


@dataclass
class T3BuildResult:
    """Everything the T3 scene build produced, and everything it changed."""

    workspace: str = ""
    scene: str = ""
    urdf_source: str = ""
    urdf_edited: str = ""
    robot_name: str = ""
    base_link: str = BASE_LINK
    ground: dict = field(default_factory=dict)
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
                "ground": dict(self.ground),
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


def urdf_efforts(urdf_path):
    """``{joint: effort}`` from the description's own ``<limit>`` tags."""
    out = {}
    try:
        root = ET.parse(urdf_path).getroot()
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


def build_t3_scene(container_dir, workspace, *, ground="box",
                   armature=ARMATURE, kp=SERVO_KP, kv=SERVO_KV,
                   spawn_z=SPAWN_Z, force_limits=True):
    """One shipped description in, one driveable MJCF scene out. Never raises.

    ``container_dir`` is ``tasks/T3_quadruped/container``. Returns a
    :class:`T3BuildResult`; anything that went wrong is in ``problems`` and the
    scene path is left empty.

    ``force_limits`` clamps every actuator to the effort the URDF declares and
    **defaults to on**, which is a deliberate divergence from the recorded
    recipe (it left them unlimited). The reason is measured, not stylistic:
    with the limits off this gait peaks at **95.4 N.m** on a thigh joint whose
    own description declares **30**, so the walk would be a demonstration that
    a robot *stronger than the one the container ships* can cross ten metres.
    Clamped, the same gait covers **31.54 m against 31.52 m** and every joint
    stays inside its declared effort -- the spikes are transient. The stricter
    configuration is therefore the default and the recipe's is
    ``force_limits=False``; the runner measures the peak force per joint
    against the declared efforts either way, so the comparison stays a
    measurement rather than an assumption.
    """
    import mujoco

    res = T3BuildResult(workspace=str(workspace), force_limited=bool(force_limits))
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
    res.urdf_edited = str(ws / "walker.urdf")
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
        'discardvisual="true" balanceinertia="true" inertiafromgeom="false"/>'
        "</mujoco> into the description",
        DOC_URDF_EXTENSIONS + "; " + DOC_COMPILER,
        detail="at the URDF default fusestatic='true' a jointless root link is "
               "ABSORBED INTO THE WORLD BODY and the compiled scene carries no "
               "body named 'base_link'. T3 grades the base by name "
               "(meta.json -> robot.declared_name), so the grader would refuse "
               "the cell. inertiafromgeom='false' keeps the declared masses, "
               "per this column's T1 mass audit"))
    res.steps.append(BuildStep(
        "urdf_floating_base",
        "added a URDF link 'world' and a joint '%s_float' of type 'floating' "
        "at z = %g m, so the robot is a free body standing on the ground"
        % (BASE_LINK, spawn_z),
        DOC_URDF_FLOATING,
        detail="the spawn height is the commanded stance solved backwards: "
               "foot sphere radius 0.022 + the commanded 0.24 m hip-to-foot "
               "distance + the 0.02 m the hips sit below the base origin = "
               "0.282 m, so this is three millimetres of settle rather than a "
               "drop test"))

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
            hinges += 1
            res.joints.append(j.name)
    res.steps.append(BuildStep(
        "mjcf_armature_and_integrator",
        "set joint/armature = %g kg.m^2 on all %d hinges and "
        "option/integrator = implicitfast at a %g s timestep"
        % (armature, hinges, TIMESTEP_S),
        DOC_ARMATURE + "; " + DOC_INTEGRATOR,
        detail="MEASURED, AND THE SINGLE MOST LIKELY FIRST BLOCKER ON ANY "
               "COLUMN: with armature 0 (the importer default, because URDF "
               "has no field for it) and an explicit integrator at this step, "
               "a kp = %g servo on a shank whose own inertia is 8.5e-4 kg.m^2 "
               "has an undamped period shorter than two steps. Told to HOLD A "
               "FIXED STANDING POSE the robot bounced across the floor at "
               "about 1 m/s, drifted 1.816 m, flipped over inside 3 s, and the "
               "peak joint tracking error was 0.42 rad -- about 100 N.m of "
               "commanded torque to stand still. With these two settings the "
               "same command holds the base at z = 0.2768 m with zero drift "
               "and all four feet down for 20 s. Lowering kp instead also "
               "works (kp = 60 was stable at the same step). Nothing warns "
               "either way, and the failure reads exactly like a physics or "
               "modelling defect" % kp))

    # -- step 3: the ground, which URDF cannot express ---------------------
    _add_ground(mujoco, spec, ground, res)

    # -- step 4: actuators, which URDF cannot express either ---------------
    _add_actuators(mujoco, spec, kp, kv, res, force_limits=force_limits)

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
            "the compiled scene carries no body named %r; T3 grades the base "
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
        "treats as massless. Set to 'false' in the extension block above"))

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
    res.steps.append(BuildStep(
        "reload_check",
        "loaded the written scene back from disk: %d bodies, %d actuators, "
        "%d joints, peak dof armature %g kg.m^2"
        % (back.nbody, back.nu, back.njnt, float(back.dof_armature.max())),
        "phase B re-runs the deliverable COLD from the file "
        "(capability-ladder-plan.md SPEC 2.3), so 'it compiled in memory' is "
        "not evidence that it is a deliverable -- and the armature is checked "
        "on the RELOADED model because it is the one setting here that no "
        "format the robot arrived in could carry"))
    return res


def _add_ground(mujoco, spec, kind, res):
    """A named static body carrying the floor, plus one light.

    Named rather than a bare world geom because T3.4 asserts that *nothing but
    the ground* touched the robot, and ``world`` as a contact partner tells a
    reader nothing about what the robot was standing on. See the module
    docstring for why the default is a finite slab.
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
                   "scene facts, not robot facts, and no importer of any "
                   "format could have supplied them. ⚠ A plane's world AABB is "
                   "±1e10 m, so the walking region derived from it is true, "
                   "unreadable, and useless to T3's arena channel")
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
            why = ("URDF has no world, no floor and no lighting: those are "
                   "scene facts, not robot facts, and no importer of any "
                   "format could have supplied them. The floor is a BOX rather "
                   "than a plane so its world-space AABB is exact -- that box "
                   "IS the walking region T3's arena channel reports, and a "
                   "bounded floor is strictly harder than an unbounded one")
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
            if force_limits else "OFF -- the recorded recipe's setting, under "
                                 "which this gait peaks at 95.4 N.m on a "
                                 "30 N.m thigh joint")),
        "URDF cannot express an actuator at all -- <transmission> names a "
        "ros_control interface, not a force -- so a URDF-only scene has nu=0 "
        "and nothing can move the robot",
        detail="the efforts the description declares are %s. The runner "
               "MEASURES the peak actuator force per joint against them and "
               "writes it into the run either way, so 'did the walk stay "
               "inside the robot's own limits' is a measurement rather than an "
               "assumption. Re-run with force_limits=False to reproduce the "
               "recorded recipe's unlimited actuators"
               % ", ".join("%s=%gN.m" % (k, v)
                           for k, v in sorted(res.efforts.items()))))


def write_build_record(res, path):
    Path(path).write_text(json.dumps(res.as_dict(), indent=2),
                          encoding="utf-8")


__all__ = ["ARMATURE", "BASE_LINK", "GROUND_BODY", "GROUND_FRICTION",
           "GROUND_PLANE_HALF", "GROUND_THICK", "GROUND_X", "GROUND_Y",
           "JOINTS", "LEGS", "SERVO_KP", "SERVO_KV", "SPAWN_Z", "TIMESTEP_S",
           "T3BuildResult", "build_t3_scene", "prepare_urdf",
           "urdf_efforts", "write_build_record"]
