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

"""**Raw ROS URDF -> a driveable MuJoCo scene**, by MuJoCo's own documented path.

T1 ships the robot as *"a URDF plus its meshes, and nothing else"*
(``docs/developer/capability-ladder-plan.md`` §2 T1). This module is the honest
answer to what that costs on MuJoCo, encoded so it can be re-read rather than
re-argued. Every step below is a step **the agent would have to take**, and each
carries the primary-source citation that says it is MuJoCo's own path and not
ours.

The documented path, quoted
---------------------------

MuJoCo's *Modeling* chapter, "URDF extensions" (``doc/modeling.rst``, anchor
``CURDF``), states the whole recipe::

    In addition to standard URDF files, MuJoCo can load files that have a
    custom (from the viewpoint of URDF) mujoco element as a child of the
    top-level element robot. This custom element can have sub-elements
    compiler, option, size with the same functionality as in MJCF, except that
    the default compiler settings are modified so as to accommodate the URDF
    modeling convention. ... This extension is also needed to specify mesh
    directories. Also note that the compiler attributes strippath, angle,
    fusestatic and discardvisual have different default values for URDF and
    MJCF.

    ... If the user wants to build models taking full advantage of MuJoCo and
    at the same time maintain URDF compatibility, we recommend the following
    procedure. Introduce extensions in the URDF as needed, load it and save it
    as MJCF. Then add information to the MJCF using include elements whenever
    possible.

So: **(1)** put a ``<mujoco><compiler .../></mujoco>`` block in the URDF,
**(2)** load and save as MJCF, **(3)** add the rest on the MJCF side. That is
exactly what :func:`build_scene` does, in that order, and the intermediate
conversion is written to disk so a reader can see the URDF half separately from
what we added.

What the URDF genuinely cannot carry, measured not assumed
----------------------------------------------------------

Four things, each verified against MuJoCo 3.8.1 on this checkout's
``husky.urdf`` (the measurements are in ``BRINGUP.md`` §3):

``package://`` mesh URIs
    MuJoCo ships no ROS package resolver. A raw load fails with
    ``Error opening file 'package://husky_description/meshes/top_plate.stl'``.
    MuJoCo's own answer is ``compiler/strippath``, which "will remove any path
    information in file names specified in the model", plus ``compiler/meshdir``
    which "instructs the compiler where to look for mesh and height field
    files". Both go in the URDF extension block. ⚠ ``strippath`` **defaults to
    false** (it used to be hardcoded true for URDF and no longer is), so this
    must be set explicitly.

COLLADA (``.dae``) visual meshes
    MuJoCo has no ``.dae`` decoder -- ``no decoder found for mesh file
    '...base_link.dae'``. On a URDF this is normally *invisible*, because
    ``compiler/discardvisual`` defaults to **true for URDF** and the visual
    geoms are dropped before their meshes are read. It becomes a hard failure
    the moment an agent sets ``discardvisual="false"`` to get a nicer picture.
    Recorded, not worked around.

a floating base
    A URDF root link has no joint to the world, and "if no joints are defined
    within a given body, that body is welded to its parent". With
    ``fusestatic`` (default **true for URDF**) the chassis is not merely welded,
    it is *absorbed into the world body*: the raw Husky compiles to 5 bodies
    (world + 4 wheels), the chassis geometry becomes world geometry, and the
    wheels spin in place for ever. Two first-party fixes exist and **both were
    verified here**: URDF's own ``<joint type="floating">`` from a dummy
    ``world`` link (MuJoCo's URDF parser honours it), and MJCF's
    ``<freejoint>`` added on the MJCF side. This module uses the URDF one,
    because it keeps the conversion a pure conversion.

actuators, a ground plane, a light
    URDF cannot express any of them. This is not a MuJoCo gap -- it is what
    URDF is -- but it is why "MuJoCo loads URDF" never means "an agent that
    ships a URDF has a scene". They are added on the MJCF side, in step 3.

Nothing here silently repairs the robot. Every edit is recorded in
:class:`BuildStep` with what changed and why, and :meth:`BuildResult.as_dict`
is written into the run directory so the grader publishes the recipe next to
the numbers.
"""

from __future__ import annotations

import json
import math
import os
import re
import shutil
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

# --- citations ---------------------------------------------------------------

DOC_URDF_EXTENSIONS = (
    "MuJoCo Modeling chapter, 'URDF extensions' "
    "(https://mujoco.readthedocs.io/en/stable/modeling.html, anchor CURDF; "
    "source doc/modeling.rst in google-deepmind/mujoco)")
DOC_COMPILER = (
    "MuJoCo XML reference, compiler element "
    "(https://mujoco.readthedocs.io/en/stable/XMLreference.html): strippath "
    "default 'false'; fusestatic and discardvisual default 'false' for MJCF "
    "and 'true' for URDF")
DOC_MODEL_EDIT = (
    "MuJoCo model-editing API (mujoco.MjSpec / mjSpec), the first-party "
    "programmatic route from a loaded model back to MJCF text: MjSpec.to_xml() "
    "and mj_saveLastXML()")

# --- defaults ----------------------------------------------------------------

# Extensions MuJoCo can actually decode. Verified negatively for .dae on 3.8.1
# ("no decoder found for mesh file"), which is the whole reason discardvisual
# matters on a ROS URDF.
DECODABLE_MESH_EXT = (".stl", ".obj", ".msh")

# The dummy link name a URDF floating joint hangs off. "world" is the ROS
# convention and MuJoCo maps it onto its own world body.
WORLD_LINK = "world"


@dataclass
class BuildStep:
    """One edit, with the citation that says whose idea it was."""

    name: str
    what: str
    citation: str = ""
    detail: str = ""
    applied: bool = True

    def as_dict(self):
        return {"step": self.name, "what": self.what, "why": self.citation,
                "detail": self.detail, "applied": bool(self.applied)}


@dataclass
class BuildResult:
    """Everything the build produced, and everything it had to change."""

    workspace: str = ""
    urdf_source: str = ""
    urdf_edited: str = ""
    mjcf_from_urdf: str = ""       # the pure URDF -> MJCF conversion
    scene: str = ""                # the deliverable: conversion + what we added
    robot_name: str = ""
    base_link: str = ""
    drive_joints: list = field(default_factory=list)
    wheel_radius_m: float = None
    half_track_m: float = None
    mass_kg_declared: float = None    # sum of the URDF's own <mass value=...>
    mass_kg_compiled: float = None    # sum of mjModel.body_mass
    steps: list = field(default_factory=list)
    problems: list = field(default_factory=list)
    meshes_referenced: list = field(default_factory=list)
    meshes_undecodable: list = field(default_factory=list)

    def as_dict(self):
        return {
            "workspace": self.workspace, "urdf_source": self.urdf_source,
            "urdf_edited": self.urdf_edited,
            "mjcf_from_urdf": self.mjcf_from_urdf, "scene": self.scene,
            "robot_name": self.robot_name, "base_link": self.base_link,
            "drive_joints": list(self.drive_joints),
            "wheel_radius_m": self.wheel_radius_m,
            "half_track_m": self.half_track_m,
            "mass_kg_declared_by_urdf": self.mass_kg_declared,
            "mass_kg_in_the_compiled_model": self.mass_kg_compiled,
            "steps": [s.as_dict() for s in self.steps],
            "problems": list(self.problems),
            "meshes_referenced": list(self.meshes_referenced),
            "meshes_undecodable": list(self.meshes_undecodable),
        }


# --- URDF reading ------------------------------------------------------------


def read_urdf(path):
    """``(robot_name, links, joints)`` from a URDF. Raises on unparseable XML.

    ``joints`` entries are ``dict(name, type, parent, child, origin_xyz)``.
    Only what the build needs; this is not a URDF library.
    """
    root = ET.parse(path).getroot()
    name = root.get("name") or ""
    links = [el.get("name") for el in root.findall("link") if el.get("name")]
    joints = []
    for el in root.findall("joint"):
        origin = el.find("origin")
        xyz = (0.0, 0.0, 0.0)
        if origin is not None and origin.get("xyz"):
            try:
                v = [float(x) for x in origin.get("xyz").split()]
                xyz = (v[0], v[1], v[2])
            except (ValueError, IndexError):
                pass
        joints.append({
            "name": el.get("name") or "",
            "type": el.get("type") or "",
            "parent": (el.find("parent").get("link")
                       if el.find("parent") is not None else None),
            "child": (el.find("child").get("link")
                      if el.find("child") is not None else None),
            "origin_xyz": xyz,
        })
    return name, links, joints


def root_link(links, joints):
    """The URDF root: a link that is never a joint's child."""
    children = {j["child"] for j in joints if j["child"]}
    for ln in links:
        if ln not in children:
            return ln
    return links[0] if links else ""


def mesh_refs(urdf_text):
    """Every ``filename="..."`` a ``<mesh>`` element names, in file order.

    Gazebo plugin ``<plugin filename="lib*.so">`` elements use the same
    attribute name, so this reads the XML rather than the text: a build that
    tried to resolve ``libgazebo_ros_control.so`` as a mesh would report a
    missing mesh that no simulator was ever asked for.
    """
    out = []
    root = ET.fromstring(urdf_text)
    for mesh in root.iter("mesh"):
        fn = mesh.get("filename")
        if fn:
            out.append(fn)
    return out


def resolve_mesh_dirs(urdf_path, refs, package_roots=()):
    """Where the referenced mesh basenames actually live.

    Returns ``(dirs, unresolved)`` -- ``dirs`` is the set of directories (as
    ``Path``) that contain at least one referenced basename, searched under the
    URDF's own package tree. ``strippath`` collapses every reference to its
    basename, so **one** directory is the case that works; more than one is a
    real trap and is reported rather than papered over.
    """
    urdf_path = Path(urdf_path)
    roots = [Path(p) for p in package_roots] or [urdf_path.parent.parent]
    dirs, unresolved = [], []
    for ref in refs:
        base = os.path.basename(ref.replace("\\", "/"))
        found = None
        for r in roots:
            for cand in r.rglob(base):
                found = cand
                break
            if found:
                break
        if found is None:
            unresolved.append(ref)
        elif found.parent not in dirs:
            dirs.append(found.parent)
    return dirs, unresolved


# --- the build ---------------------------------------------------------------

_MUJOCO_BLOCK = (
    '  <!-- Added by the capability-ladder MuJoCo column. MuJoCo\'s own\n'
    '       documented URDF extension: "This custom element can have\n'
    '       sub-elements compiler, option, size ... This extension is also\n'
    '       needed to specify mesh directories." (Modeling / URDF extensions) -->\n'
    '  <mujoco>\n'
    '    <compiler meshdir="%(meshdir)s" strippath="true" fusestatic="false"\n'
    '              discardvisual="true" balanceinertia="true"\n'
    '              inertiafromgeom="false"/>\n'
    '  </mujoco>\n')

_FLOAT_JOINT = (
    '  <!-- Added by the capability-ladder MuJoCo column. A URDF root link has\n'
    '       no joint to the world, so MuJoCo welds it (and with fusestatic it\n'
    '       absorbs it into the world body outright). URDF\'s own floating\n'
    '       joint type is the in-format fix and MuJoCo\'s URDF parser honours\n'
    '       it (verified on 3.8.1). -->\n'
    '  <link name="%(world)s"/>\n'
    '  <joint name="%(jname)s" type="floating">\n'
    '    <parent link="%(world)s"/>\n'
    '    <child link="%(base)s"/>\n'
    '    <origin rpy="0 0 0" xyz="0 0 %(z)s"/>\n'
    '  </joint>\n')


def build_scene(description_dir, workspace, *, spawn_z=0.25,
                ground_half_size=60.0, wheel_kv=60.0, urdf_path=None):
    """Raw URDF tree in, driveable MJCF scene out. Never raises on physics.

    ``description_dir`` is the shipped robot package (it is **copied**, never
    edited in place). ``workspace`` is where the copy and every generated file
    land. Returns a :class:`BuildResult`; an XML or compile failure is recorded
    in ``problems`` and the corresponding output path is left empty.
    """
    import mujoco                       # local: the module must import w/o it

    res = BuildResult(workspace=str(workspace))
    src = Path(description_dir)
    ws = Path(workspace)
    ws.mkdir(parents=True, exist_ok=True)
    pkg = ws / src.name
    if pkg.exists():
        shutil.rmtree(pkg)
    shutil.copytree(src, pkg)
    res.steps.append(BuildStep(
        "copy", "copied the shipped robot description into the run workspace",
        "the ladder ships the robot as files; editing them in place would "
        "mutate the fixture every later run reads",
        detail=str(pkg)))

    urdf = Path(urdf_path) if urdf_path else None
    if urdf is None:
        cands = sorted(pkg.rglob("*.urdf"))
        if not cands:
            res.problems.append("no .urdf under %s" % pkg)
            return res
        urdf = cands[0]
    else:
        urdf = pkg / urdf.relative_to(src)
    res.urdf_source = str(src / urdf.relative_to(pkg))
    res.urdf_edited = str(urdf)

    text = urdf.read_text(encoding="utf-8")
    try:
        robot_name, links, joints = read_urdf(urdf)
    except ET.ParseError as exc:
        res.problems.append("the URDF is not well-formed XML: %r" % exc)
        return res
    res.robot_name = robot_name
    base = root_link(links, joints)
    res.base_link = base

    # -- step 1: the mesh directory --------------------------------------
    try:
        refs = mesh_refs(text)
    except ET.ParseError as exc:
        res.problems.append("could not read mesh references: %r" % exc)
        refs = []
    res.meshes_referenced = list(dict.fromkeys(refs))
    res.meshes_undecodable = sorted(
        {r for r in refs
         if os.path.splitext(r)[1].lower() not in DECODABLE_MESH_EXT})
    dirs, unresolved = resolve_mesh_dirs(urdf, refs, package_roots=[pkg])
    if unresolved:
        res.problems.append(
            "%d mesh reference(s) could not be located under %s: %s"
            % (len(unresolved), pkg.name, ", ".join(unresolved[:4])))
    if len(dirs) > 1:
        res.problems.append(
            "referenced meshes live in %d different directories (%s). "
            "compiler/strippath collapses every reference to its basename, so "
            "a single meshdir cannot serve them and this robot needs the "
            "references rewritten instead."
            % (len(dirs), ", ".join(d.name for d in dirs)))
    meshdir = os.path.relpath(dirs[0], urdf.parent).replace("\\", "/") + "/" \
        if dirs else "./"

    block = _MUJOCO_BLOCK % {"meshdir": meshdir}
    res.steps.append(BuildStep(
        "urdf_mujoco_extension",
        "inserted a <mujoco><compiler/></mujoco> element into the URDF: "
        "meshdir=%r strippath=true fusestatic=false discardvisual=true "
        "balanceinertia=true inertiafromgeom=false" % meshdir,
        DOC_URDF_EXTENSIONS + "; " + DOC_COMPILER,
        detail="without strippath+meshdir the load fails outright on the "
               "package:// URIs; discardvisual is left at the URDF default so "
               "the undecodable .dae visual meshes are never opened (%d of the "
               "%d referenced meshes are undecodable by MuJoCo)"
               % (len(res.meshes_undecodable), len(res.meshes_referenced))))

    # -- step 2: the floating base ---------------------------------------
    float_joint = "%s_floating_base" % (base or "base")
    if WORLD_LINK in links:
        res.problems.append(
            "the URDF already declares a %r link; the floating joint was not "
            "added and the base may be welded" % WORLD_LINK)
        fj = ""
    else:
        fj = _FLOAT_JOINT % {"world": WORLD_LINK, "jname": float_joint,
                             "base": base, "z": "%g" % spawn_z}
        res.steps.append(BuildStep(
            "urdf_floating_base",
            "added a URDF link %r and a joint %r of type 'floating' so the "
            "chassis is not welded to the world" % (WORLD_LINK, float_joint),
            "URDF has no floating-base concept for its root link and MuJoCo "
            "welds a body with no joint to its parent; MuJoCo's URDF parser "
            "does honour joint type='floating' (verified, 3.8.1)",
            detail="without it the raw Husky compiles to 5 bodies -- world "
                   "plus 4 wheels -- with the chassis absorbed into world "
                   "geometry and nq=4"))

    new_text = re.sub(r"(<robot\b[^>]*>)", lambda m: m.group(1) + "\n" + block
                      + fj, text, count=1)
    urdf.write_text(new_text, encoding="utf-8")

    # Re-parse the edited URDF with a strict parser before handing it to
    # MuJoCo. This is not belt-and-braces: MuJoCo's own docs say "while MJCF
    # models are checked against a custom XML schema by the parser, URDF models
    # are not", and it is measurably more permissive than a strict reader --
    # during bring-up it silently compiled a file whose XML comment contained a
    # double hyphen, which ElementTree rejects outright. A file only one of the
    # two readers accepts is a file whose provenance nobody can check.
    try:
        ET.fromstring(new_text)
    except ET.ParseError as exc:
        res.problems.append(
            "the edited URDF is not well-formed XML (%r). MuJoCo may still "
            "load it -- its URDF path is not schema-checked -- but no other "
            "tool will, so the edit is reported rather than trusted" % exc)

    # -- step 3: URDF -> MJCF, then add what URDF cannot say --------------
    try:
        spec = mujoco.MjSpec.from_file(str(urdf))
    except (ValueError, RuntimeError) as exc:
        res.problems.append("MuJoCo refused the URDF: %s" % exc)
        return res

    conv = urdf.parent / ("%s_from_urdf.xml" % (robot_name or "robot"))
    try:
        conv.write_text(spec.to_xml(), encoding="utf-8")
        res.mjcf_from_urdf = str(conv)
        res.steps.append(BuildStep(
            "save_as_mjcf",
            "saved the loaded URDF back out as MJCF (%s)" % conv.name,
            DOC_URDF_EXTENSIONS + " -- 'load it and save it as MJCF'; "
            + DOC_MODEL_EDIT))
    except (ValueError, RuntimeError, OSError) as exc:
        res.problems.append("MJCF save failed: %r" % exc)

    drive = [j["name"] for j in joints
             if j["type"] in ("continuous", "revolute")]
    res.drive_joints = drive

    _add_ground(spec, ground_half_size, res)
    _add_actuators(spec, drive, wheel_kv, res)

    try:
        model = spec.compile()
    except (ValueError, RuntimeError) as exc:
        res.problems.append("the assembled scene did not compile: %s" % exc)
        return res

    res.wheel_radius_m, res.half_track_m = _drive_geometry(mujoco, model, drive,
                                                           res)
    _mass_audit(model, urdf, res)

    scene = urdf.parent / ("%s_scene.xml" % (robot_name or "robot"))
    try:
        scene.write_text(spec.to_xml(), encoding="utf-8")
        res.scene = str(scene)
    except (ValueError, RuntimeError, OSError) as exc:
        res.problems.append("scene save failed: %r" % exc)
    return res


def _add_ground(spec, half, res):
    """A named static body carrying an infinite plane, plus a light.

    The ground is its own **named body** rather than a bare world geom on
    purpose: T1.4 asks for a contact naming the robot and the surface under it,
    and ``world`` as a surface name tells a reader nothing about what the robot
    was riding on.
    """
    import mujoco
    try:
        ground = spec.worldbody.add_body()
        ground.name = "ground"
        g = ground.add_geom()
        g.name = "ground_plane"
        g.type = mujoco.mjtGeom.mjGEOM_PLANE
        g.size = [half, half, 0.05]
        g.rgba = [0.35, 0.37, 0.35, 1.0]
        light = spec.worldbody.add_light()
        light.name = "sun"
        light.pos = [0.0, 0.0, 12.0]
        light.dir = [0.0, 0.0, -1.0]
        light.type = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
        res.steps.append(BuildStep(
            "mjcf_ground_and_light",
            "added a static body 'ground' carrying an infinite plane geom "
            "(half-size %g m) and one directional light" % half,
            "URDF has no world, no ground and no lighting: those are scene "
            "facts, not robot facts, so no importer of any format could have "
            "supplied them"))
    except (AttributeError, ValueError, RuntimeError) as exc:
        res.problems.append("could not add the ground: %r" % exc)


def _add_actuators(spec, joints, kv, res):
    """One joint-velocity actuator per drive joint.

    MJCF's ``<velocity>`` shortcut is an affine gain/bias pair
    (``gainprm[0] = kv``, ``biasprm[2] = -kv``); the spec API has no shortcut
    elements, so the pair is set directly. Same actuator either way -- the
    saved MJCF is what a reader can check.
    """
    import mujoco
    added = []
    for jn in joints:
        try:
            a = spec.add_actuator()
            a.name = "vel_%s" % jn
            a.target = jn
            a.trntype = mujoco.mjtTrn.mjTRN_JOINT
            a.gaintype = mujoco.mjtGain.mjGAIN_AFFINE
            a.biastype = mujoco.mjtBias.mjBIAS_AFFINE
            a.gainprm[0] = float(kv)
            a.biasprm[2] = -float(kv)
            added.append(a.name)
        except (AttributeError, ValueError, RuntimeError) as exc:
            res.problems.append("could not add an actuator for %r: %r"
                                % (jn, exc))
    if added:
        res.steps.append(BuildStep(
            "mjcf_actuators",
            "added %d joint-velocity actuators (kv=%g): %s"
            % (len(added), kv, ", ".join(added)),
            "URDF cannot express an actuator at all -- <transmission> names a "
            "ros_control interface, not a force -- so a URDF-only scene has "
            "nu=0 and nothing can drive the robot"))


def _drive_geometry(mujoco, model, drive_joints, res):
    """``(wheel_radius_m, half_track_m)`` read off the compiled model.

    Both are **measurements of the robot the agent was given**, not constants
    typed from a datasheet: the radius is the wheel geom's own size and the
    half-track is the wheel body's lateral offset. A controller built on typed
    constants is a controller that silently drives the wrong robot.
    """
    radii, tracks = [], []
    for jn in drive_joints:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        if jid < 0:
            continue
        bid = int(model.jnt_bodyid[jid])
        tracks.append(abs(float(model.body_pos[bid][1])))
        for gid in range(int(model.body_geomnum[bid])):
            g = int(model.body_geomadr[bid]) + gid
            if int(model.geom_type[g]) == int(mujoco.mjtGeom.mjGEOM_CYLINDER):
                radii.append(float(model.geom_size[g][0]))
    r = float(sum(radii) / len(radii)) if radii else None
    b = float(sum(tracks) / len(tracks)) if tracks else None
    if r is None:
        res.problems.append(
            "no cylinder geom on any drive-joint body: the wheel radius could "
            "not be measured off the model and the controller has no way to "
            "convert a body speed into a wheel speed")
    if r is not None and radii and (max(radii) - min(radii)) > 1e-6:
        res.problems.append(
            "the drive wheels are not all the same radius (%.4f..%.4f m); the "
            "mean was used" % (min(radii), max(radii)))
    return r, b


def urdf_declared_mass(urdf_path):
    """Sum of the URDF's own ``<inertial><mass value=.../>`` entries, kg."""
    try:
        root = ET.parse(urdf_path).getroot()
    except ET.ParseError:
        return None
    total = 0.0
    for el in root.iter("mass"):
        try:
            total += float(el.get("value"))
        except (TypeError, ValueError):
            continue
    return total


def _mass_audit(model, urdf, res):
    """Compare the compiled mass with the mass the URDF actually declares.

    **The trap this exists to make visible.** ``compiler/inertiafromgeom``
    defaults to ``"auto"``: use an explicit ``<inertial>`` where the link has
    one, otherwise *infer* mass and inertia from the link's geoms at
    ``geom/density`` (default 1000 kg/m^3). URDF says the opposite -- a link
    with no ``<inertial>`` is massless. On this checkout's Husky the difference
    is not subtle and it is silent: ``base_link`` carries two collision boxes
    and no inertial, so the default infers **116.55 kg** for it and the robot
    compiles at **175.84 kg** against the URDF's declared **56.58 kg**, a
    factor of 3.1. Nothing warns. Setting ``inertiafromgeom="false"`` in the
    URDF extension block reproduces the declared 56.58 kg exactly (measured,
    MuJoCo 3.8.1).

    A ladder column that shipped the 175 kg robot would be measuring a
    different vehicle from every other column, so the fix is applied and the
    audit is published beside it rather than being taken on trust.
    """
    res.mass_kg_compiled = float(sum(float(x) for x in model.body_mass))
    res.mass_kg_declared = urdf_declared_mass(urdf)
    res.steps.append(BuildStep(
        "mass_audit",
        "compiled total mass %.3f kg against the URDF's declared %.3f kg"
        % (res.mass_kg_compiled, res.mass_kg_declared or float("nan")),
        "compiler/inertiafromgeom defaults to 'auto' -- infer mass/inertia "
        "from geoms (geom/density default 1000 kg/m^3) for any link with no "
        "<inertial>, which URDF treats as massless. Set to 'false' in the "
        "extension block above",
        detail=("measured on this Husky: 'auto' compiles a 175.838 kg robot "
                "(base_link inferred at 116.546 kg from its two collision "
                "boxes); 'false' compiles the declared 56.582 kg. No warning "
                "is emitted either way")))
    d = res.mass_kg_declared
    if d and abs(res.mass_kg_compiled - d) > 0.02 * d:
        res.problems.append(
            "the compiled mass (%.3f kg) is more than 2%% off the URDF's "
            "declared mass (%.3f kg); the robot being simulated is not the "
            "robot the description declares"
            % (res.mass_kg_compiled, d))


def write_build_record(res, path):
    """The build recipe, as JSON, next to the run it produced."""
    Path(path).write_text(json.dumps(res.as_dict(), indent=2),
                          encoding="utf-8")


def wheel_targets(v, w, radius, half_track):
    """``(left, right)`` wheel angular velocities for a skid-steer body twist.

    rad/s, from ``v`` m/s forward and ``w`` rad/s yaw rate. Plain kinematics,
    kept here so the controller in :mod:`runner` has no geometry of its own.
    """
    if not radius:
        return 0.0, 0.0
    b = float(half_track or 0.0)
    return ((float(v) - float(w) * b) / float(radius),
            (float(v) + float(w) * b) / float(radius))


def wrap_pi(a):
    """``a`` wrapped into (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


__all__ = ["BuildResult", "BuildStep", "DECODABLE_MESH_EXT",
           "DOC_COMPILER", "DOC_MODEL_EDIT", "DOC_URDF_EXTENSIONS",
           "build_scene", "mesh_refs", "read_urdf", "resolve_mesh_dirs",
           "root_link", "wheel_targets", "wrap_pi", "write_build_record"]
