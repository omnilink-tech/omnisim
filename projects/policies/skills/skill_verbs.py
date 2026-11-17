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

"""The skill-library verb table -- ONE definition, two front doors.

Both CLIs build their argparse subcommands from :func:`register` here:

* ``python -m omnisim policy``            -- the stable front door (delegates execution)
* ``projects/policies/skills/skill_lib.py`` -- the compatibility runner (executes)

They used to declare the verb set twice: skill_lib.py owned the real subparsers,
and ``omnisim/policy/cli.py`` kept a hand-copied ``PASSTHROUGH`` name set that it
forwarded but never registered. The forwarding worked, so ``omnisim policy list``
ran -- but ``omnisim policy --help`` listed only the six natively-implemented
verbs, so the documented front door read as if ``list`` / ``preview`` / ``train`` /
``sequence`` did not exist. A second hand-maintained list is how that happened;
this module is why it cannot happen again.

Deps: ``argparse`` only. ``omnisim policy`` loads this file directly (never
``skill_lib``) so drawing a help screen costs no manifest discovery, no
``sys.path`` mutation, and no heavy import.
"""

from __future__ import annotations

import argparse
from typing import Callable, Iterable

# (name, help, add_arguments). The callables receive the freshly created
# subparser; `p.add_argument(...)` returns an action, and a tuple of them is
# discarded -- the calls are the point.
_VERBS: tuple[tuple[str, str, Callable[[argparse.ArgumentParser], object]], ...] = (
    ("list", "List every skill + BATON sequence in the catalogue.",
     lambda p: None),
    ("show", "Print one skill/sequence manifest, ghost summary and assembled env.",
     lambda p: p.add_argument("name")),
    ("validate", "Validate every versioned manifest and contract ghost LUT.",
     lambda p: None),
    ("ghost", "Describe/validate ghost LUTs (the pre-training reference gate).",
     lambda p: (p.add_argument("lut", nargs="?"),
                p.add_argument("--all", action="store_true"),
                p.add_argument("--strict", action="store_true"))),
    ("preview", "Launch the ghost hologram for a skill (design -> show -> agree).",
     lambda p: (p.add_argument("name"),
                p.add_argument("--duration", type=int, default=30),
                p.add_argument("--dry-run", action="store_true"))),
    ("train", "Assemble the recipe env and launch the in-engine trainer.",
     lambda p: (p.add_argument("name"),
                p.add_argument("--iters", type=int, default=300),
                p.add_argument("--gui", default="headless"),
                p.add_argument("--dry-run", action="store_true"))),
    ("run", "Deploy a single skill solo.",
     lambda p: (p.add_argument("name"),
                p.add_argument("--duration", type=int, default=60),
                p.add_argument("--gui", default="gui"),
                p.add_argument("--throw", action="store_true"),
                p.add_argument("--dry-run", action="store_true"))),
    ("sequence", "Deploy a BATON sequence (box_delivery, walk_turn_walk, ...).",
     lambda p: (p.add_argument("name"),
                p.add_argument("--duration", type=int, default=None),
                p.add_argument("--gui", default=None),
                p.add_argument("--dry-run", action="store_true"))),
    ("verify-demos", "Prove the assembled env == the reproduced demo shell scripts.",
     lambda p: None),
    ("audit", "Run every static release gate in one command.",
     lambda p: None),
    ("benchmark", "Run/score the policy-block acceptance suite.",
     lambda p: p.add_argument("benchmark_args", nargs=argparse.REMAINDER)),
    ("index", "Regenerate registry.json from filesystem discovery.",
     lambda p: None),
    ("adapt", "Resample a skill's ghost to a target cadence (cross-cadence blend).",
     lambda p: (p.add_argument("name"),
                p.add_argument("--to-nb", type=int, required=True),
                p.add_argument("--out", default=None))),
    ("blendable", "Report whether two skills can element-wise blend, and what is needed.",
     lambda p: (p.add_argument("a"), p.add_argument("b"))),
    ("handover", "Show a sequence's resolved per-edge warm/cold handover plan.",
     lambda p: p.add_argument("name")),
    ("freeze", "Promote a training run's captured env lock into the manifest.",
     lambda p: (p.add_argument("name"),
                p.add_argument("--from", dest="from_env", default=None),
                p.add_argument("--checkpoint", default=None))),
)


def names(skip: Iterable[str] = ()) -> tuple[str, ...]:
    """Every verb name, in declaration order, minus `skip`."""
    skipped = set(skip)
    return tuple(name for name, _, _ in _VERBS if name not in skipped)


def register(sub: argparse._SubParsersAction, skip: Iterable[str] = ()) -> tuple[str, ...]:
    """Add every verb (minus `skip`) to an existing `add_subparsers()` action.

    Returns the names registered, so a caller can derive its dispatch set from
    what it actually registered rather than from a second literal list.
    """
    skipped = set(skip)
    added: list[str] = []
    for name, help_text, add_arguments in _VERBS:
        if name in skipped:
            continue
        add_arguments(sub.add_parser(name, help=help_text))
        added.append(name)
    return tuple(added)
