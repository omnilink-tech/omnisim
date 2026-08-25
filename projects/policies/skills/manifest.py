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

"""Skill / Sequence / Profile manifests + deploy-env assembly for the OmniSim skill library.

A **skill** (walk, turn-in-place, carry-box, stand, climb-stairs, ...) is today
scattered across five disconnected artifacts: a ghost lut JSON, an ephemeral
validator verdict, a frozen env-var bundle buried in a shell script, a champion
checkpoint in ``runs/*.pt``, and provenance lore in memory notes. This module binds
them into ONE versioned manifest (``skill.json`` v2) and reuses the already-generic
recipe trainer / corridor / WBMATCH / BATON / deploy stack unchanged.

Three artifact types:

* **Profile** (``profiles/<name>.json``)   -- the runtime env identical across every
  demo (obs family, harness/crane gains, PD, the recipe deploy pymod). Extracted from
  the common core of the hand-written demo scripts.
* **SkillManifest** (``<class>/<skill>/skill.json``) -- identity + ghost + policy +
  baton role + train recipe + deploy env + verification record.
* **SequenceManifest** (``sequences/<name>.json``) -- a BATON demo referencing skills
  by name: {profile, world, primary, skills[], arbiter, env}.

The load-bearing operation is :func:`assemble_deploy_env`, which merges
profile -> primary-skill -> cyclic specialists (``BATON_SPECIALISTS``) -> solo-swap
skills (``BATON_TURN_*``) -> arbiter (``BATON_COURSE`` | ``WALK_SCHEDULE``) ->
sequence overrides -> ``WALK_WORLD``, producing the exact ``KEY=VALUE`` bundle the
demo shell scripts pass to ``run_walk_rl.sh``. That equivalence is the correctness
gate (see ``tests``/``skill_lib.py verify-demos``): the manifests must reproduce the
proven scripts byte-for-byte before the manifest path replaces them.

Pure-python, no torch / no engine import -- safe to run anywhere.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

SKILLS_DIR = Path(__file__).resolve().parent
REPO_ROOT = SKILLS_DIR.parents[2]   # projects/policies/skills -> repo root

# The canonical recipe deploy entry point (the #1 silent-failure trap if wrong).
RECIPE_DEPLOY_PYMOD = "projects.policies.training.g1_walk_recipe:g1_walk_recipe_deploy"
RECIPE_TRAIN_PYMOD = "projects.policies.training.g1_walk_recipe:g1_walk_recipe_step"


# --------------------------------------------------------------------------- #
#  small helpers
# --------------------------------------------------------------------------- #
def _load_json(path: Path) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a JSON object")
    return data


def _stringify_env(env: dict[str, Any]) -> dict[str, str]:
    """Coerce every value to the string form ``run_walk_rl.sh`` will export.

    Bools are lowercased is-irrelevant here (the scripts use ``1``); ints/floats are
    emitted verbatim so ``0`` stays ``0`` and ``0.100`` stays ``0.100`` -- therefore
    numeric env values are stored as STRINGS in the manifests (preserving the exact
    literal the proven scripts used).
    """
    out: dict[str, str] = {}
    for k, v in env.items():
        if isinstance(v, bool):
            out[k] = "1" if v else "0"
        else:
            out[k] = str(v)
    return out


# --------------------------------------------------------------------------- #
#  Profile
# --------------------------------------------------------------------------- #
@dataclass
class Profile:
    name: str
    raw: dict[str, Any]

    @classmethod
    def load(cls, name: str) -> "Profile":
        p = SKILLS_DIR / "profiles" / f"{name}.json"
        if not p.exists():
            raise FileNotFoundError(f"profile '{name}' not found at {p}")
        return cls(name=name, raw=_load_json(p))

    @property
    def env(self) -> dict[str, str]:
        return _stringify_env(self.raw.get("env", {}))


# --------------------------------------------------------------------------- #
#  SkillManifest
# --------------------------------------------------------------------------- #
@dataclass
class SkillManifest:
    path: Path
    raw: dict[str, Any]

    @classmethod
    def load(cls, path: str | Path) -> "SkillManifest":
        p = Path(path)
        return cls(path=p, raw=_load_json(p))

    # ---- identity ----
    @property
    def name(self) -> str:
        return self.raw["name"]

    @property
    def title(self) -> str:
        return self.raw.get("title", "")

    @property
    def klass(self) -> str:
        return self.raw.get("class", "")

    @property
    def robots(self) -> list[str]:
        return list(self.raw.get("robots", []))

    @property
    def kind(self) -> str:               # rl | deterministic
        return self.raw.get("kind", "rl")

    @property
    def method(self) -> str:             # shadowing | deterministic-overlay
        return self.raw.get("method", "")

    @property
    def status(self) -> str:             # verified | experimental | open
        return self.raw.get("status", "experimental")

    @property
    def motion_class(self) -> str:       # cyclic | sequence | static
        return self.raw.get("motion_class", "")

    # ---- sub-blocks (raw dicts) ----
    @property
    def ghost(self) -> dict[str, Any]:
        return self.raw.get("ghost", {})

    @property
    def policy(self) -> dict[str, Any]:
        return self.raw.get("policy", {})

    @property
    def baton(self) -> dict[str, Any]:
        return self.raw.get("baton", {})

    @property
    def train(self) -> dict[str, Any]:
        return self.raw.get("train", {})

    @property
    def deploy(self) -> dict[str, Any]:
        return self.raw.get("deploy", {})

    # ---- convenience ----
    @property
    def blend(self) -> str:              # cyclic | solo_swap | (deterministic: "")
        return self.baton.get("blend", "")

    @property
    def baton_mode(self) -> str:
        """The short mode name the arbiter switches to (may differ from name)."""
        return self.baton.get("mode", self.name)

    @property
    def checkpoint(self) -> str:
        # `null` is meaningful: a deterministic HOLD has no policy (go2_stand). It must render as
        # the empty field in a BATON_SPECIALISTS entry ("stand||<lut>|0"), never as "None".
        return self.policy.get("checkpoint") or ""

    @property
    def ghost_lut(self) -> str:
        return self.ghost.get("lut", "")

    @property
    def ghost_format(self) -> str:
        # "lut" (the phase-indexed json) | "npz" (the quadruped research lineage)
        return self.ghost.get("format", "lut")

    @property
    def attractor(self) -> str:
        return self.baton.get("attractor", "locomotion")

    @property
    def deploy_run(self) -> str:
        """How `run <skill>` launches this skill.

        recipe     -> run_walk_rl.sh deploy (the in-engine Shadowing recipe skills)
        world      -> launch the .wbt whose controller is baked in (re-host / model-based)
        powershell -> the deterministic humanoid_stand_deploy PowerShell launcher
        """
        m = self.deploy.get("run")
        if m:
            return m
        if self.kind == "deterministic":
            return "powershell"
        if self.method == "shadowing":
            return "recipe"
        return "world"

    # ---- deploy-env contributions ----
    def primary_env(self) -> dict[str, str]:
        """Env emitted when this skill is the BATON primary policy."""
        return _stringify_env(self.deploy.get("primary_env", {}))

    def specialist_entry(self) -> str:
        """The ``mode|ckpt|lut|vx`` token for a cyclic BATON specialist.

        Paths stay REPO-RELATIVE — they are the portable form, they match the
        hand-written demo scripts key-for-key (``verify-demos`` asserts that),
        and they do not bake one machine's checkout root into a launch env.
        Resolving them is the reader's job: ``baton.setup`` anchors a relative
        path at ``OMNISIM_HOME``, because a deploy controller's cwd is its own
        directory, not the repo root.
        """
        vx = self.baton.get("vx", "0")
        return f"{self.baton_mode}|{self.checkpoint}|{self.ghost_lut}|{vx}"

    def solo_swap_env(self) -> dict[str, str]:
        """Env emitted when this skill is a solo-swap specialist (e.g. the turn)."""
        return _stringify_env(self.deploy.get("solo_swap_env", {}))

    # ---- validation ----
    def validate(self) -> list[str]:
        issues: list[str] = []
        for req in ("schema", "name", "class", "robots", "kind", "method", "status",
                    "motion_class"):
            if req not in self.raw:
                issues.append(f"missing required field '{req}'")
        if self.raw.get("schema") != 2:
            issues.append(f"schema must be 2, got {self.raw.get('schema')!r}")
        if self.klass not in ("humanoid", "quadruped"):
            issues.append(f"class must be humanoid|quadruped, got {self.klass!r}")
        if not self.robots:
            issues.append("robots must name at least one compatible robot")
        if self.kind not in ("rl", "deterministic"):
            issues.append(f"kind must be rl|deterministic, got {self.kind!r}")
        if self.method not in ("shadowing", "residual-rl", "unitree-rehost",
                               "deterministic-overlay"):
            issues.append(f"unknown policy method {self.method!r}")
        if self.kind == "deterministic" and self.method != "deterministic-overlay":
            issues.append("deterministic skills must declare method='deterministic-overlay'")
        if self.kind == "rl" and self.method == "deterministic-overlay":
            issues.append("rl skills cannot declare method='deterministic-overlay'")
        if self.status not in ("verified", "experimental", "open"):
            issues.append(f"status must be verified|experimental|open, got {self.status!r}")
        if self.motion_class not in ("cyclic", "sequence", "static"):
            issues.append(f"motion_class must be cyclic|sequence|static, got {self.motion_class!r}")
        if self.status == "verified" and not isinstance(self.raw.get("verification"), dict):
            issues.append("verified skill requires a structured 'verification' record")
        issues += self._name_follows_convention()
        if self.kind == "rl":
            if not self.motion_class:
                issues.append("rl skill missing 'motion_class'")
            if not self.checkpoint:
                issues.append("rl skill missing policy.checkpoint")
            if self.blend not in ("cyclic", "solo_swap"):
                issues.append(f"rl skill baton.blend must be cyclic|solo_swap, got {self.blend!r}")
            # a phase-indexed json ghost is required only for Shadowing-lut skills;
            # re-host / residual / generated-ghost skills legitimately have none.
            if self.method == "shadowing" and self.ghost_format == "lut" and not self.ghost_lut:
                issues.append("shadowing(lut) skill missing ghost.lut")
            # A registered skill is a deployable contract, so its bound artifacts must exist.
            if self.ghost_lut and not (REPO_ROOT / self.ghost_lut).exists():
                issues.append(f"ghost.lut not found on disk: {self.ghost_lut}")
            if self.checkpoint and not (REPO_ROOT / self.checkpoint).exists():
                issues.append(f"policy.checkpoint not found on disk: {self.checkpoint}")
            for block, key in (("ghost", "preview_world"), ("train", "launcher"),
                               ("deploy", "world"), ("deploy", "controller"),
                               ("deploy", "launcher")):
                value = (self.raw.get(block) or {}).get(key)
                if value and not (REPO_ROOT / value).exists():
                    issues.append(f"{block}.{key} not found on disk: {value}")
            # ⛔ THE MANIFEST MAY NOT OVERCLAIM ITS GHOST'S VERDICT.
            # `ghost.validator` was PROSE a human typed next to the artifact -- and it drifted:
            # carry_box and climb_stairs both said "PASS" while ghost_validator.py said WARN on
            # the very lut they point at. The verdict is now STAMPED INTO the lut
            # (ghost_validator.py --stamp), and the claim is checked against the stamp here.
            issues += self._verdict_agrees()
        else:
            for key in ("spec", "world", "launcher"):
                value = self.raw.get(key)
                if value and not (REPO_ROOT / value).exists():
                    issues.append(f"{key} not found on disk: {value}")
        return issues

    def _name_follows_convention(self) -> list[str]:
        """THE SKILL-NAMING RULE, enforced rather than hoped for.

        A single-robot skill is named ``<robot>_<skill>``; a genuinely multi-robot skill is
        named ``<skill>``. That is the whole rule.

        Why it needs enforcing: the humanoid skills used to be named for the skill alone
        (``walk``, ``carry_box``) with the G1 as the UNNAMED DEFAULT, while the quadruped ones
        were robot-prefixed (``go2_walk``, ``omniquad_walk``). In a flat registry namespace that
        means ``walk`` is the G1's forever -- an H1 Shadowing walk has no name left to take,
        and ``h1_walk`` was already spent on the re-host. The unnamed default is exactly the
        habit that made this whole stack G1-shaped; a robot that is never named is a robot that
        is never generalized away from.

        The METHOD stays a FIELD (`method`), not a name: go2_walk and go2_shadow_walk differ by
        method, and `method` is what you must branch on -- never the string in the name.
        """
        name, robots = str(self.raw.get("name", "")), self.robots
        if len(robots) == 1:
            pfx = f"{robots[0]}_"
            if not name.startswith(pfx):
                return [f"name {name!r} must start with {pfx!r}: a single-robot skill is named "
                        f"<robot>_<skill> (no unnamed default robot)"]
        elif len(robots) > 1:
            for r in robots:
                if name.startswith(f"{r}_"):
                    return [f"name {name!r} is robot-prefixed but the skill claims {len(robots)} "
                            f"robots {robots} -- drop the prefix or split the skill"]
        return []

    def _verdict_agrees(self) -> list[str]:
        if not self.ghost_lut:
            return []
        p = REPO_ROOT / self.ghost_lut
        if not p.exists():
            return []
        try:
            stamped = ((json.loads(p.read_text(encoding="utf-8")) or {})
                       .get("validator") or {}).get("verdict")
        except Exception:                                    # noqa: BLE001
            return []
        if not stamped:
            return [f"warn: ghost lut carries no stamped verdict "
                    f"(run: python projects/policies/training/ghost_validator.py {self.ghost_lut} --stamp)"]
        claim = str((self.raw.get("ghost") or {}).get("validator") or "").strip().upper()
        if claim and not claim.startswith(stamped.upper()):
            return [f"ghost.validator says {claim!r} but the lut is stamped {stamped!r} "
                    f"-- the manifest is overclaiming its own ghost"]
        return []


# --------------------------------------------------------------------------- #
#  SequenceManifest
# --------------------------------------------------------------------------- #
@dataclass
class SequenceManifest:
    path: Path
    raw: dict[str, Any]

    @classmethod
    def load(cls, path: str | Path) -> "SequenceManifest":
        p = Path(path)
        return cls(path=p, raw=_load_json(p))

    @property
    def name(self) -> str:
        return self.raw["name"]

    @property
    def title(self) -> str:
        return self.raw.get("title", "")

    @property
    def status(self) -> str:
        return self.raw.get("status", "experimental")

    @property
    def profile(self) -> str:
        return self.raw.get("profile", "g1_shadow_deploy")

    @property
    def world(self) -> str:
        return self.raw["world"]

    # ---- how this sequence DEPLOYS (see assemble_deploy_env) ----
    @property
    def deploy_kind(self) -> str:
        """"recipe" (the G1 in-engine pymod, the default) | "world" (a launcher + a .wbt)."""
        return (self.raw.get("deploy") or {}).get("kind", "recipe")

    @property
    def launcher(self) -> str:
        """The script that opens the world, for deploy_kind == "world"."""
        return (self.raw.get("deploy") or {}).get("launcher", "")

    @property
    def primary(self) -> str:
        return self.raw["primary"]

    @property
    def skills(self) -> list[str]:
        return list(self.raw.get("skills", []))

    @property
    def arbiter(self) -> dict[str, Any]:
        return self.raw.get("arbiter", {})

    @property
    def env(self) -> dict[str, str]:
        return _stringify_env(self.raw.get("env", {}))

    @property
    def duration_default(self) -> int:
        return int(self.raw.get("duration_default", 120))

    @property
    def gui_default(self) -> str:
        return self.raw.get("gui_default", "gui")

    @property
    def init_mode_file(self) -> bool:
        return bool(self.raw.get("init_mode_file", False))

    @property
    def reproduces(self) -> str:
        return self.raw.get("reproduces", "")

    def validate(self, registry: "Registry") -> list[str]:
        issues: list[str] = []
        for req in ("schema", "name", "class", "robot", "status", "profile", "world", "primary",
                    "skills", "arbiter"):
            if req not in self.raw:
                issues.append(f"missing required field '{req}'")
        if self.raw.get("schema") != 2:
            issues.append(f"schema must be 2, got {self.raw.get('schema')!r}")
        klass = self.raw.get("class", "")
        robot = self.raw.get("robot", "")
        if klass not in ("humanoid", "quadruped"):
            issues.append(f"class must be humanoid|quadruped, got {klass!r}")
        if not robot:
            issues.append("robot must name the sequence robot")
        if self.status not in ("verified", "experimental", "open"):
            issues.append(f"status must be verified|experimental|open, got {self.status!r}")
        if self.status == "verified" and not isinstance(self.raw.get("verification"), dict):
            issues.append("verified sequence requires a structured 'verification' record")
        try:
            Profile.load(self.profile)
        except FileNotFoundError as e:
            issues.append(str(e))
        if self.primary and self.primary not in registry.skills:
            issues.append(f"primary skill '{self.primary}' not in registry")
        for s in self.skills:
            if s not in registry.skills:
                issues.append(f"referenced skill '{s}' not in registry")
        if len(self.skills) != len(set(self.skills)):
            issues.append("skills contains duplicate references")
        for sname in [self.primary] + self.skills:
            skill = registry.skills.get(sname)
            if not skill:
                continue
            if skill.klass != klass:
                issues.append(f"skill '{sname}' class {skill.klass!r} is incompatible with "
                              f"sequence class {klass!r}")
            if robot not in skill.robots:
                issues.append(f"skill '{sname}' does not support sequence robot {robot!r}")
        ak = self.arbiter.get("kind")
        if ak not in ("course", "schedule", None):
            issues.append(f"arbiter.kind must be course|schedule, got {ak!r}")
        if ak and not self.arbiter.get("value"):
            issues.append("arbiter.value must not be empty")
        mode_map = {registry.skills[s].baton_mode for s in [self.primary] + self.skills
                    if s in registry.skills}
        for mode in dict.fromkeys(ordered_modes(self)):
            if mode not in mode_map:
                issues.append(f"arbiter mode '{mode}' has no primary/referenced skill")
        if self.world and not (REPO_ROOT / self.world).exists():
            issues.append(f"world not found on disk: {self.world}")
        if self.deploy_kind not in ("recipe", "world"):
            issues.append(f"deploy.kind must be recipe|world, got {self.deploy_kind!r}")
        if self.deploy_kind == "world":
            if not self.launcher:
                issues.append("world deploy requires deploy.launcher")
            elif not (REPO_ROOT / self.launcher).exists():
                issues.append(f"deploy.launcher not found on disk: {self.launcher}")
        if self.reproduces and not (REPO_ROOT / self.reproduces).exists():
            issues.append(f"reproduced script not found on disk: {self.reproduces}")
        return issues


# --------------------------------------------------------------------------- #
#  Registry
# --------------------------------------------------------------------------- #
@dataclass
class Registry:
    skills: dict[str, SkillManifest] = field(default_factory=dict)
    sequences: dict[str, SequenceManifest] = field(default_factory=dict)
    issues: list[str] = field(default_factory=list)

    @classmethod
    def discover(cls, root: Path | None = None) -> "Registry":
        root = root or SKILLS_DIR
        reg = cls()
        for p in sorted(root.glob("*/*/skill.json")):
            try:
                m = SkillManifest.load(p)
                if m.name in reg.skills:
                    reg.issues.append(
                        f"duplicate skill name {m.name!r}: {reg.skills[m.name].path} and {p}")
                    continue
                reg.skills[m.name] = m
            except Exception as e:  # noqa: BLE001
                reg.issues.append(f"failed to load {p}: {e}")
        for p in sorted((root / "sequences").glob("*.json")):
            try:
                s = SequenceManifest.load(p)
                if s.name in reg.sequences:
                    reg.issues.append(
                        f"duplicate sequence name {s.name!r}: {reg.sequences[s.name].path} and {p}")
                    continue
                reg.sequences[s.name] = s
            except Exception as e:  # noqa: BLE001
                reg.issues.append(f"failed to load {p}: {e}")
        return reg


# --------------------------------------------------------------------------- #
#  per-edge handover resolution  (the stand-attractor-lock law as data)
# --------------------------------------------------------------------------- #
def _seg_mode(token: str) -> str:
    """Map an arbiter segment token to a BATON mode: 'walkto,4,0' -> 'walk'."""
    v = token.split(",")[0].strip()
    if v.endswith("to"):
        v = v[:-2]
    return v


def ordered_modes(seq: SequenceManifest) -> list[str]:
    """The ordered list of BATON modes a sequence steps through (from its arbiter)."""
    val = seq.arbiter.get("value", "")
    if not val:
        return [seq.primary]
    kind = seq.arbiter.get("kind")
    if kind == "course":
        return [_seg_mode(s) for s in val.split(";") if s.strip()]
    if kind == "schedule":
        return [seg.split(":")[0].strip() for seg in val.split(",") if seg.strip()]
    return [seq.primary]


def resolve_handover(seq: SequenceManifest, registry: Registry) -> dict:
    """Resolve warm/cold per transition edge from skill ATTRACTORS, not lore.

    The stand-attractor-lock law: a warm all-LSTM handover is safe only INTO an
    attractor-compatible regime; entering a *locomotion* (cyclic) skill immediately
    after a *stand* locks the incoming policy in the stand attractor (it marches in
    place) -> that edge must go COLD. A solo-swap skill (turn/climb) manages its own
    hidden and is marked 'solo'. Returns per-edge decisions + the ``global_cold`` flag
    the current single-knob deploy consumes (BATON_COLD_HIDDEN).
    """
    # mode -> skill (primary + referenced specialists), keyed by each skill's baton mode
    mode_to_skill: dict[str, SkillManifest] = {}
    for sname in [seq.primary] + seq.skills:
        sk = registry.skills.get(sname)
        if sk:
            mode_to_skill[sk.baton_mode] = sk
    modes = ordered_modes(seq)
    edges: list[dict] = []
    for prev, cur in zip(modes, modes[1:]):
        ps, cs = mode_to_skill.get(prev), mode_to_skill.get(cur)
        if cs is None:
            edges.append({"edge": f"{prev}->{cur}", "decision": "unknown", "reason": "unmapped mode"})
            continue
        if cs.blend == "solo_swap":
            edges.append({"edge": f"{prev}->{cur}", "decision": "solo",
                          "reason": "solo-swap specialist manages its own hidden (cold both ways)"})
            continue
        prev_att = ps.attractor if ps else "locomotion"
        if prev_att == "stand" and cs.attractor == "locomotion":
            edges.append({"edge": f"{prev}->{cur}", "decision": "cold",
                          "reason": "entering locomotion after a stand -> warm state locks the stand attractor"})
        else:
            edges.append({"edge": f"{prev}->{cur}", "decision": "warm",
                          "reason": "attractor-compatible handover"})
    global_cold = any(e["decision"] == "cold" for e in edges)
    return {"modes": modes, "edges": edges, "global_cold": global_cold}


# --------------------------------------------------------------------------- #
#  freeze a training bundle into a skill manifest  (#3)
# --------------------------------------------------------------------------- #
def freeze_train_bundle(skill_path: str | Path, env: dict[str, str],
                        checkpoint: str | None = None) -> None:
    """Record the EXACT training env used for a run into the skill manifest.

    Flips ``train.status`` from 'reconstructed' to 'frozen' and stores the verbatim
    env under ``train.frozen_env`` (and optionally updates ``policy.checkpoint``), so a
    re-train reproduces the champion instead of guessing the recipe from notes.
    """
    p = Path(skill_path)
    raw = _load_json(p)
    tr = raw.setdefault("train", {})
    tr["frozen_env"] = dict(env)
    tr["status"] = "frozen"
    if checkpoint:
        raw.setdefault("policy", {})["checkpoint"] = checkpoint
    p.write_text(json.dumps(raw, indent=2) + "\n", encoding="utf-8")


# --------------------------------------------------------------------------- #
#  deploy-env assembly  (the load-bearing operation)
# --------------------------------------------------------------------------- #
def assemble_deploy_env(seq: SequenceManifest, registry: Registry) -> dict[str, str]:
    """Merge a sequence into the exact ``KEY=VALUE`` env bundle its deploy needs.

    Precedence (later wins): profile < primary skill < solo-swap skills <
    sequence.env < world. Cyclic specialists are collapsed into one
    ``BATON_SPECIALISTS`` string. This is what the hand-written demo scripts encode
    inline; the manifest factors it out.

    ⭐ TWO DEPLOY KINDS (2026-07-13). This used to emit one shape only -- the KEY=VAL bundle for
    ``run_walk_rl.sh``, i.e. the G1's in-engine recipe -- because ``RECIPE_DEPLOY_PYMOD`` was the
    only deploy the sequence layer knew. That made the whole sequence layer G1-only: a quadruped
    BATON demo deploys as a WORLD + CONTROLLER (an ONNX controller in its own process, no crane,
    no recipe pymod), so it could not be expressed as a sequence at all, no matter that the
    arbiter driving it is the same one.

        deploy.kind = "recipe"  (default)  -> run_walk_rl.sh <dur> <tag> deploy <gui> KEY=VAL...
        deploy.kind = "world"              -> bash <deploy.launcher> ... with the env EXPORTED

    The arbiter key differs with it: the recipe reads ``WALK_SCHEDULE``, the generic BATON
    library reads ``BATON_SCHEDULE``. Everything else -- profile, specialists, handover
    derivation, overrides -- is shared, which is the point.
    """
    env: dict[str, str] = {}
    world_kind = seq.deploy_kind == "world"

    # 1. runtime profile
    env.update(Profile.load(seq.profile).env)

    # 2. primary policy
    primary = registry.skills[seq.primary]
    env.update(primary.primary_env())

    # 3. referenced skills: cyclic -> BATON_SPECIALISTS ; solo_swap -> BATON_TURN_* env
    specialists: list[str] = []
    for sname in seq.skills:
        skill = registry.skills[sname]
        if skill.blend == "cyclic":
            specialists.append(skill.specialist_entry())
        elif skill.blend == "solo_swap":
            env.update(skill.solo_swap_env())
    if specialists:
        env["BATON_SPECIALISTS"] = ";".join(specialists)

    # 4. arbiter
    ak = seq.arbiter.get("kind")
    av = seq.arbiter.get("value", "")
    if ak == "course":
        env["BATON_COURSE"] = av
    elif ak == "schedule":
        # the recipe's own scheduler reads WALK_SCHEDULE; the BATON library reads BATON_SCHEDULE
        env["BATON_SCHEDULE" if world_kind else "WALK_SCHEDULE"] = av

    # 4b. per-edge handover (#2): auto-derive the cold-hidden knob from skill attractors
    # (the stand-attractor-lock law as data). A sequence may still override in seq.env.
    plan = resolve_handover(seq, registry)
    if plan["global_cold"]:
        env["BATON_COLD_HIDDEN"] = "1"

    # 5. sequence-level overrides (settle/decel tuning, morph, mode file, ...).
    # An explicit empty-string value UNSETS a derived key (e.g. BATON_COLD_HIDDEN
    # when a verified demo deliberately runs the warm handover) so the assembled
    # env can stay byte-identical to the ground-truth demo script.
    env.update(seq.env)
    env = {k: v for k, v in env.items() if v != ""}

    # 6. the world. The recipe takes it as WALK_WORLD (run_walk_rl.sh loads it); a `world` deploy
    #    IS the world -- the launcher opens the .wbt whose controller line is already baked in.
    if not world_kind:
        env["WALK_WORLD"] = seq.world
    return env


def assemble_solo_env(skill: SkillManifest, registry: "Registry | None" = None,
                      profile_name: str = "g1_shadow_deploy") -> dict[str, str]:
    """Env to deploy a single RL skill by itself.

    A cyclic skill deploys as its own primary; a solo-swap skill deploys the base
    walker as primary and swaps itself in (mirrors the ``turn_solo`` shape). For a
    faithful single-skill demo prefer a sequence manifest; this is a convenience.
    """
    env: dict[str, str] = {}
    env.update(Profile.load(profile_name).env)
    if skill.blend == "cyclic":
        # deploy this skill as the primary using its own ghost + checkpoint
        env["RES_POLICY"] = skill.checkpoint
        env["GHOST_LUT_JSON"] = skill.ghost_lut
        env.update(skill.primary_env())
    else:
        # SOLO-SWAP: the BASE WALKER is the primary policy and this skill swaps in on top of it --
        # exactly the run_turn_solo.sh shape. So the base's OWN primary env (its checkpoint, its ghost
        # lut, its corridor) must be present, or there is no policy and no ghost to swap away from.
        #
        # ⛔ It was not. Until 2026-07-13 this branch emitted the SWAP KEYS ALONE, so
        # `skill_lib run g1_turn_in_place` / `g1_climb_stairs` / `g1_carry_box` rendered a launch with
        # no RES_POLICY, no GHOST_LUT_JSON and no GHOST_RESIDUAL: a Shadowing checkpoint started with
        # nothing to shadow and its corridor switched OFF. It did not error -- it just deployed
        # garbage (the "skills" then collapsed on the floor and looked like broken policies).
        # `verify-demos` never caught it because it only verifies SEQUENCES, never solo runs.
        base_name = skill.baton.get("solo_base", "g1_walk")
        base = registry.skills.get(base_name) if registry is not None else None
        if base is None:
            raise ValueError(
                f"solo-swap skill '{skill.name}' rides on a base policy ('{base_name}') and cannot be "
                f"deployed without it. Pass the registry to assemble_solo_env(), or declare a different "
                f"baton.solo_base in the manifest. (Emitting the swap keys alone yields a corridor-free, "
                f"ghost-free launch -- silently broken, which is what this used to do.)")
        env["RES_POLICY"] = base.checkpoint
        env["GHOST_LUT_JSON"] = base.ghost_lut
        env.update(base.primary_env())
        env.update(skill.solo_swap_env())
    return env


def render_run_argv(env: dict[str, str], tag: str, duration: int, gui: str,
                    seq: "SequenceManifest | None" = None) -> list[str]:
    """Build the launch argv for a sequence.

    recipe (default): ``bash run_walk_rl.sh <dur> <tag> deploy <gui> KEY=VAL ...`` -- the env
      rides as positional KEY=VAL args, which is the contract that launcher was written to.
    world:            ``bash <launcher> <dur>`` -- the env is EXPORTED by the caller instead
      (see skill_lib.cmd_sequence), because a world launcher reads it from the environment.
    """
    if seq is not None and seq.deploy_kind == "world":
        return ["bash", seq.launcher, str(duration)]
    argv = [
        "bash", "projects/policies/training/run_walk_rl.sh",
        str(duration), tag, "deploy", gui,
    ]
    for k in sorted(env):
        argv.append(f"{k}={env[k]}")
    return argv
