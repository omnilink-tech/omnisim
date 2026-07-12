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
        return self.policy.get("checkpoint", "")

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
        """The ``mode|ckpt|lut|vx`` token for a cyclic BATON specialist."""
        vx = self.baton.get("vx", "0")
        return f"{self.baton_mode}|{self.checkpoint}|{self.ghost_lut}|{vx}"

    def solo_swap_env(self) -> dict[str, str]:
        """Env emitted when this skill is a solo-swap specialist (e.g. the turn)."""
        return _stringify_env(self.deploy.get("solo_swap_env", {}))

    # ---- validation ----
    def validate(self) -> list[str]:
        issues: list[str] = []
        for req in ("name", "class", "kind", "status"):
            if req not in self.raw:
                issues.append(f"missing required field '{req}'")
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
            # existence checks (warn-level: checkpoints are frequently uncommitted)
            if self.ghost_lut and not (REPO_ROOT / self.ghost_lut).exists():
                issues.append(f"warn: ghost.lut not found on disk: {self.ghost_lut}")
            if self.checkpoint and not (REPO_ROOT / self.checkpoint).exists():
                issues.append(f"warn: policy.checkpoint not found on disk (uncommitted?): {self.checkpoint}")
        return issues


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
        for req in ("name", "world", "primary"):
            if req not in self.raw:
                issues.append(f"missing required field '{req}'")
        try:
            Profile.load(self.profile)
        except FileNotFoundError as e:
            issues.append(str(e))
        if self.primary and self.primary not in registry.skills:
            issues.append(f"primary skill '{self.primary}' not in registry")
        for s in self.skills:
            if s not in registry.skills:
                issues.append(f"referenced skill '{s}' not in registry")
        ak = self.arbiter.get("kind")
        if ak not in ("course", "schedule", None):
            issues.append(f"arbiter.kind must be course|schedule, got {ak!r}")
        if self.world and not (REPO_ROOT / self.world).exists():
            issues.append(f"warn: world not found on disk: {self.world}")
        return issues


# --------------------------------------------------------------------------- #
#  Registry
# --------------------------------------------------------------------------- #
@dataclass
class Registry:
    skills: dict[str, SkillManifest] = field(default_factory=dict)
    sequences: dict[str, SequenceManifest] = field(default_factory=dict)

    @classmethod
    def discover(cls, root: Path | None = None) -> "Registry":
        root = root or SKILLS_DIR
        reg = cls()
        for p in sorted(root.glob("*/*/skill.json")):
            try:
                m = SkillManifest.load(p)
                reg.skills[m.name] = m
            except Exception as e:  # noqa: BLE001
                print(f"[skill-registry] failed to load {p}: {e}")
        for p in sorted((root / "sequences").glob("*.json")):
            try:
                s = SequenceManifest.load(p)
                reg.sequences[s.name] = s
            except Exception as e:  # noqa: BLE001
                print(f"[skill-registry] failed to load {p}: {e}")
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
    """Merge a sequence into the exact ``KEY=VALUE`` env bundle for ``run_walk_rl.sh``.

    Precedence (later wins): profile < primary skill < solo-swap skills <
    sequence.env < WALK_WORLD. Cyclic specialists are collapsed into one
    ``BATON_SPECIALISTS`` string. This is what the hand-written demo scripts encode
    inline; the manifest factors it out.
    """
    env: dict[str, str] = {}

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
        env["WALK_SCHEDULE"] = av

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

    # 6. world
    env["WALK_WORLD"] = seq.world
    return env


def assemble_solo_env(skill: SkillManifest, profile_name: str = "g1_shadow_deploy") -> dict[str, str]:
    """Env to deploy a single RL skill by itself (best-effort).

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
        env.update(skill.solo_swap_env())
    return env


def render_run_argv(env: dict[str, str], tag: str, duration: int, gui: str) -> list[str]:
    """Build the ``bash run_walk_rl.sh <dur> <tag> deploy <gui> KEY=VAL ...`` argv."""
    argv = [
        "bash", "projects/policies/training/run_walk_rl.sh",
        str(duration), tag, "deploy", gui,
    ]
    for k in sorted(env):
        argv.append(f"{k}={env[k]}")
    return argv
