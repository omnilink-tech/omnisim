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

"""omnisim validate-urdf — is this robot description physically realisable?

`scripts/dev/urdf_import.py --strict` has carried these checks for a while, but
only as a side effect of *converting* a URDF. That is the wrong shape for the
two cases that matter most: auditing a robot you have no intention of importing,
and running as a CI gate on a directory of them. This verb is the same
predicates behind a plain exit code.

It answers a narrower question than "does this load". A description can parse,
satisfy every requirement the URDF specification states, import here without a
single warning, and still describe a body that cannot exist -- and the engines
disagree about what to do with it. MuJoCo refuses such a model outright, PyBullet
accepts it and silently zeroes the tensor, and OmniSim's own Newton path will
build it. So the same file "works" in one stack and not another, and the author
finds out from a bug report rather than from a check.

Tiers, because "invalid" is not one thing and a single verdict is wrong for
somebody:

``topology``
    The kinematic graph is a tree: parent/child name links that exist, exactly
    one root, no cycles, unique names, and ``<mimic>`` targets a real movable
    joint.

``physics``
    The numbers describe a realisable body. Mass positive where there is
    geometry; the inertia tensor symmetric positive definite *and* satisfying
    the triangle inequality on its principal moments; actuated joints declaring
    a non-zero effort and velocity; joint ranges non-empty.

``assets``
    Referenced meshes resolve. Off by default -- it is the only tier whose
    answer depends on where the file is checked out rather than on the model.

The two physics checks worth stating explicitly, because both have cost real
debugging time in descriptions shipped by robot vendors:

**Positive definiteness is necessary but not sufficient.** The eigenvalues of an
inertia tensor are its principal moments, and for any real distribution of mass
they satisfy ``J_a + J_b >= J_c`` -- each is an integral of a sum of two squared
coordinates, so no arrangement of matter can violate it. A tensor can be
symmetric, have three strictly positive eigenvalues, and still describe no body.
``diag(1, 1, 3)`` is the minimal example.

**A declared zero is a declaration, not a blank.** URDF spells "unlimited" by
*omitting* an attribute, so ``effort="0"`` states that the joint can apply no
torque. An importer that helpfully substitutes a default hides the defect in
exactly the tool where it would have been caught -- which is why OmniSim's own
synthetic-gain substitution now warns instead of staying quiet.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

#: Tiers in report order, and in order of increasing opinion.
TIERS = ("topology", "physics", "assets")

#: Checked unless the caller says otherwise. ``assets`` is excluded because a
#: missing mesh is a property of the checkout, not of the description.
DEFAULT_TIERS = ("topology", "physics")

#: A radius of gyration beyond this many metres is not a robot link; it is
#: almost always an inertia tensor left in g*mm^2 on a kg*m^2 model. Only
#: reported when extreme, so the check stays quiet on real robots.
_IMPLAUSIBLE_GYRATION_M = 100.0


def _import_urdf_module():
    """Load ``scripts/dev/urdf_import.py`` as the single source of these checks.

    Deliberately not a copy: the predicates there are the ones that have been
    calibrated against real descriptions, and a second implementation would
    drift from them silently.
    """
    scripts_dir = str(REPO_ROOT / "scripts" / "dev")
    if scripts_dir not in sys.path:
        sys.path.insert(0, scripts_dir)
    import urdf_import  # noqa: E402  (path is set up immediately above)

    return urdf_import


class Finding:
    """One problem, with the measured value and the bound it violates."""

    __slots__ = ("tier", "check", "element", "message")

    def __init__(self, tier: str, check: str, element: str, message: str):
        self.tier = tier
        self.check = check
        self.element = element
        self.message = message

    def __str__(self) -> str:
        return f"[{self.tier}/{self.check}] {self.element}: {self.message}"

    def as_dict(self) -> dict:
        return {
            "tier": self.tier,
            "check": self.check,
            "element": self.element,
            "message": self.message,
        }


def _has_geometry(link) -> bool:
    """Whether a link carries any collision or visual geometry.

    A link with no geometry is the ordinary URDF idiom for a coordinate frame --
    a sensor mount, a tool centre point -- and zero mass is correct for one.
    Flagging those would bury the real findings, so the mass checks are
    conditioned on this.
    """
    return bool(getattr(link, "collisions", None) or getattr(link, "visuals", None))


def _check_topology(robot, findings: list[Finding]) -> None:
    link_names = [l.name for l in robot.links.values()]
    known = set(link_names)

    for name in sorted({n for n in link_names if link_names.count(n) > 1}):
        findings.append(Finding("topology", "duplicate_link_name", name,
                                "declared more than once"))
    joint_names = [j.name for j in robot.joints]
    for name in sorted({n for n in joint_names if joint_names.count(n) > 1}):
        findings.append(Finding("topology", "duplicate_joint_name", name,
                                "declared more than once"))

    parent_joint_of: dict[str, str] = {}
    for j in robot.joints:
        for role, ref in (("parent", j.parent), ("child", j.child)):
            if ref and ref not in known:
                findings.append(Finding(
                    "topology", "broken_link_reference", j.name,
                    f"{role} '{ref}' is not a declared link"))
        if j.child:
            if j.child in parent_joint_of:
                findings.append(Finding(
                    "topology", "multiple_parents", j.child,
                    f"is the child of both '{parent_joint_of[j.child]}' and "
                    f"'{j.name}'; a URDF kinematic graph must be a tree"))
            else:
                parent_joint_of[j.child] = j.name

    roots = [n for n in known if n not in parent_joint_of]
    if not roots and known:
        findings.append(Finding(
            "topology", "no_root_link", "<model>",
            "every link is the child of a joint, so the graph contains a cycle "
            "and has no base link"))
    elif len(roots) > 1:
        shown = sorted(roots)[:6]
        findings.append(Finding(
            "topology", "multiple_root_links", "<model>",
            f"{len(roots)} links have no parent joint ({', '.join(shown)}"
            f"{', ...' if len(roots) > 6 else ''}); a URDF must have exactly one"))

    # Cycles: walk childward-to-rootward from each link; a repeat is a loop.
    joint_by_child = {j.child: j for j in robot.joints if j.child}
    reported: set[str] = set()
    for start in sorted(known):
        seen: set[str] = set()
        node = start
        while node in joint_by_child:
            if node in seen:
                if node not in reported:
                    reported.add(node)
                    findings.append(Finding(
                        "topology", "cycle", node,
                        "is part of a closed kinematic loop; URDF cannot express "
                        "one, and a loop-closing joint leaves the whole world "
                        "without physics under Newton"))
                break
            seen.add(node)
            node = joint_by_child[node].parent

    joint_by_name = {j.name: j for j in robot.joints}
    for j in robot.joints:
        target = getattr(j, "mimic_joint", None)
        if not target:
            continue
        if target not in joint_by_name:
            findings.append(Finding(
                "topology", "mimic_broken_reference", j.name,
                f"mimics '{target}', which is not a declared joint"))
        elif target == j.name:
            findings.append(Finding("topology", "mimic_self", j.name, "mimics itself"))
        elif getattr(joint_by_name[target], "mimic_joint", None):
            findings.append(Finding(
                "topology", "mimic_chain", j.name,
                f"mimics '{target}', which is itself a mimic joint; consumers "
                "differ on whether they resolve chained mimics"))
        elif joint_by_name[target].type == "fixed":
            findings.append(Finding(
                "topology", "mimic_fixed_target", j.name,
                f"mimics '{target}', which is a fixed joint and never moves"))


def _check_physics(robot, findings: list[Finding], urdf) -> None:
    links = list(robot.links.values())
    declared = [l for l in links if l.inertial is not None]
    with_geometry = [l for l in links if _has_geometry(l)]

    # A description in which *no* link declares an <inertial> is a coherent
    # thing -- a visualisation-only model -- and reporting every link would bury
    # the real findings under a choice the author made on purpose. A model where
    # *some* links declare inertia and others do not is a different matter, so
    # there the per-link finding stands.
    partial = 0 < len(declared) < len(with_geometry)
    if not declared and with_geometry:
        findings.append(Finding(
            "physics", "no_inertial_data", "<model>",
            "no link declares an <inertial>; the model is renderable but cannot "
            "be simulated"))

    for link in links:
        inertial = link.inertial
        if inertial is None:
            if partial and _has_geometry(link):
                findings.append(Finding(
                    "physics", "missing_inertial", link.name,
                    "carries geometry but declares no <inertial>, while other "
                    "links in this model do"))
            continue

        mass = inertial.mass
        if mass is not None:
            if mass < 0.0:
                findings.append(Finding("physics", "negative_mass", link.name,
                                        f"mass = {mass}"))
            elif mass == 0.0 and _has_geometry(link):
                findings.append(Finding(
                    "physics", "zero_mass_with_geometry", link.name,
                    "mass = 0 but the link carries geometry; a massless body "
                    "with collision geometry is not simulable"))

        ixx, ixy, ixz = inertial.ixx, inertial.ixy, inertial.ixz
        iyy, iyz, izz = inertial.iyy, inertial.iyz, inertial.izz
        components = (ixx, ixy, ixz, iyy, iyz, izz)
        if not all(math.isfinite(v) for v in components):
            findings.append(Finding("physics", "non_finite_inertia", link.name,
                                    "inertia contains nan or inf"))
            continue
        if not any(components):
            # "no <inertia> tag" and "an <inertia> tag of zeros" are different
            # defects and deserve different messages: the first is an omission
            # every consumer defaults differently, the second is an assertion
            # that the body has no rotational inertia at all.
            if not mass:
                continue
            if not getattr(inertial, "inertia_declared", True):
                findings.append(Finding(
                    "physics", "missing_inertia_tag", link.name,
                    f"declares mass = {mass} but no <inertia>; the URDF schema "
                    "requires one inside <inertial>, and consumers do not agree "
                    "on what to substitute"))
            else:
                findings.append(Finding(
                    "physics", "zero_inertia_with_mass", link.name,
                    f"inertia is declared all-zero but mass = {mass}"))
            continue

        moments = urdf.principal_moments(ixx, ixy, ixz, iyy, iyz, izz)
        scale = max(abs(m) for m in moments) or 1.0
        if min(moments) <= 0.0:
            # Report a numerically-zero smallest eigenvalue as singular rather
            # than negative: an all-equal placeholder tensor lands at ~-1e-18,
            # and calling that "negative inertia" is how a checker loses its
            # reader.
            singular = abs(min(moments)) < scale * 1e-9
            findings.append(Finding(
                "physics",
                "inertia_singular" if singular else "inertia_not_positive_definite",
                link.name,
                "principal moments ({}) are {}".format(
                    ", ".join(f"{m:.6g}" for m in moments),
                    "rank-deficient" if singular else "not all strictly positive")))
            continue

        violated, _ = urdf.inertia_violates_triangle_inequality(
            ixx, ixy, ixz, iyy, iyz, izz)
        if violated:
            a, b, c = sorted(moments)
            findings.append(Finding(
                "physics", "inertia_triangle_inequality", link.name,
                "principal moments ({}) violate J_a + J_b >= J_c "
                "({:.6g} < {:.6g}, short by {:.1f}%); the tensor is positive "
                "definite but describes no physical body".format(
                    ", ".join(f"{m:.6g}" for m in moments), a + b, c,
                    100.0 * (1.0 - (a + b) / c))))

        if mass and mass > 0.0:
            gyration = math.sqrt(max(moments) / mass)
            if gyration > _IMPLAUSIBLE_GYRATION_M:
                findings.append(Finding(
                    "physics", "implausible_radius_of_gyration", link.name,
                    f"radius of gyration is {gyration:.4g} m for a mass of "
                    f"{mass} kg, which is larger than any plausible link; check "
                    "the units of the inertia tensor"))

    # A whole file declaring effort=0 everywhere is an unpopulated field, not a
    # claim about every joint, so only flag it when the file disagrees with
    # itself.
    actuated = [j for j in robot.joints
                if j.type in ("revolute", "prismatic", "continuous")]
    efforts = [j.effort for j in actuated if j.effort is not None]
    effort_is_convention = bool(efforts) and all(e == 0.0 for e in efforts)
    velocities = [j.velocity for j in actuated if j.velocity is not None]
    velocity_is_convention = bool(velocities) and all(v == 0.0 for v in velocities)

    for joint in actuated:
        if getattr(joint, "mimic_joint", None):
            # A mimic joint is kinematically driven; zero effort is correct.
            continue
        if joint.effort == 0.0 and not effort_is_convention:
            findings.append(Finding(
                "physics", "zero_effort", joint.name,
                "declares effort = 0, i.e. the joint can apply no torque; URDF "
                "expresses 'unlimited' by omitting the attribute"))
        elif joint.effort is not None and joint.effort < 0.0:
            findings.append(Finding("physics", "negative_effort", joint.name,
                                    f"effort = {joint.effort}"))
        if joint.velocity == 0.0 and not velocity_is_convention:
            findings.append(Finding(
                "physics", "zero_velocity", joint.name,
                "declares velocity = 0, i.e. the joint can never move"))
        elif joint.velocity is not None and joint.velocity < 0.0:
            findings.append(Finding("physics", "negative_velocity", joint.name,
                                    f"velocity = {joint.velocity}"))

        if joint.type in ("revolute", "prismatic"):
            lo, hi = joint.lower, joint.upper
            if lo is not None and hi is not None:
                if hi < lo:
                    findings.append(Finding(
                        "physics", "inverted_limits", joint.name,
                        f"upper ({hi}) is below lower ({lo})"))
                elif hi == lo:
                    findings.append(Finding(
                        "physics", "zero_width_range", joint.name,
                        f"lower == upper == {lo}, so the joint is immobile; it "
                        "is probably meant to be fixed"))


def _check_assets(robot, findings: list[Finding]) -> None:
    for link in robot.links.values():
        for element in list(link.collisions) + list(link.visuals):
            geom = getattr(element, "geometry", None)
            mesh = getattr(geom, "mesh_path", None) if geom is not None else None
            if not mesh:
                continue
            if not Path(mesh).is_file():
                findings.append(Finding(
                    "assets", "mesh_not_found", link.name,
                    f"'{mesh}' does not resolve to a file"))


def validate_file(path: Path, tiers=DEFAULT_TIERS) -> list[Finding]:
    """Parse one description and return its findings, in tier order."""
    urdf = _import_urdf_module()
    robot = urdf.parse_urdf(Path(path))
    findings: list[Finding] = []
    if "topology" in tiers:
        _check_topology(robot, findings)
    if "physics" in tiers:
        _check_physics(robot, findings, urdf)
    if "assets" in tiers:
        _check_assets(robot, findings)
    return findings


def add_parser(sub) -> None:
    """Register the verb on omnisim's subparser (see omnisim/cli.py)."""
    p = sub.add_parser(
        "validate-urdf",
        help="Check a robot description is physically realisable (exit 1 on a finding).",
    )
    p.add_argument("input", nargs="+", help="One or more .urdf files.")
    p.add_argument(
        "--tiers", default=",".join(DEFAULT_TIERS),
        help=f"Comma-separated subset of {{{','.join(TIERS)}}} (default: %(default)s).")
    p.add_argument("--check-meshes", action="store_true",
                   help="Also run the 'assets' tier (mesh files must resolve).")
    p.add_argument("--json", action="store_true", help="Emit JSON instead of text.")
    p.add_argument("--quiet", "-q", action="store_true",
                   help="Print nothing; use the exit code.")
    p.set_defaults(func=main)
    return p


def main(args: argparse.Namespace) -> int:
    """0 if every file passed, 1 if any had a finding, 2 on a read error."""
    tiers = [t.strip() for t in args.tiers.split(",") if t.strip()]
    if args.check_meshes and "assets" not in tiers:
        tiers.append("assets")
    unknown = sorted(set(tiers) - set(TIERS))
    if unknown:
        print(f"omnisim validate-urdf: unknown tier(s) {unknown}; "
              f"choose from {list(TIERS)}", file=sys.stderr)
        return 2

    worst = 0
    results = []
    for name in args.input:
        try:
            findings = validate_file(Path(name), tiers=tiers)
        except Exception as exc:  # noqa: BLE001 - any parse failure is a read error
            results.append({"file": name, "error": f"{type(exc).__name__}: {exc}"})
            if not (args.quiet or args.json):
                print(f"{name}: could not be read: {type(exc).__name__}: {exc}",
                      file=sys.stderr)
            worst = 2
            continue

        results.append({
            "file": name,
            "ok": not findings,
            "tiers": tiers,
            "findings": [f.as_dict() for f in findings],
        })
        if findings and worst < 1:
            worst = 1
        if args.quiet or args.json:
            continue
        if not findings:
            print(f"{name}: OK ({', '.join(tiers)})")
        else:
            print(f"{name}: {len(findings)} finding(s)")
            for tier in TIERS:
                for f in findings:
                    if f.tier == tier:
                        print(f"  {f}")

    if args.json:
        print(json.dumps({"results": results, "exit": worst}, indent=1))
    return worst
