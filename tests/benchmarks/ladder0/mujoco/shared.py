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

"""shared.py -- locate and import the ladder's shared contract.

The MuJoCo arm owns NO ground truth.  Every scene constant, every expected
value and every tolerance comes from the ladder's shared module, which lives
one directory up and is owned by another lane.  This file is the single place
that knows how to find it, so a rename there costs one edit here.  It is also
this arm's module loader: :func:`sibling` is how every file in this directory
reaches the others.

Three mechanics are load-bearing and are not incidental style:

1. **Nothing here touches ``sys.path``.**  This arm lives in a directory called
   ``mujoco/``, so any ``sys.path`` entry pointing at its parent makes
   ``ladder0/mujoco`` shadow the real ``mujoco`` package -- ``import mujoco``
   then resolves to this directory and the arm cannot import the simulator it
   exists to drive.  It is also why this directory deliberately has no
   ``__init__.py``.  (Measured, not theorised: the first run of this arm died
   on ``module 'mujoco' has no attribute '__version__'``.)

2. **Sibling modules are loaded by path under an ARM-QUALIFIED name**
   (``ladder0_mujoco_scenes``, never ``scenes``).  All three arms ship a
   module called ``worldgen``, and two of them shipped one called ``run``;
   ``sys.modules`` is process-global, so a bare ``import worldgen`` in the
   second arm the shared runner loads resolves to the FIRST arm's generator.
   That arm then generates, and measures, another simulator's scenes while
   reporting them as its own -- and every number in the row is internally
   consistent, so no assertion in this ladder can see it.  ``run_ladder.py``
   now refuses an arm holding a foreign module; this is the other half of it.

3. **An already-loaded contract module is REUSED, never re-executed.**  The
   shared runner imports ``rungs`` before it loads any arm.  Re-exec'ing the
   file under the same ``sys.modules`` key would leave two distinct module
   objects for one contract, so the runner's ``analysis`` would be judging
   against a different object than this arm's ``spec`` -- identical values
   today, and an invisible split the moment anything patches one of them (the
   self-test does exactly that).

If the contract cannot be found the arm refuses to run rather than inventing
its own numbers -- an arm that quietly fell back to constants it made up would
produce a table that looks comparable to the other two and is not.
"""

from __future__ import annotations

import importlib.util
import os
import sys

HERE = os.path.abspath(os.path.dirname(__file__))
LADDER0 = os.path.dirname(HERE)

CONTRACT_NAMES = ("rungs", "spec")      # newest name first
ARM_PREFIX = "ladder0_mujoco_"          # every module this arm registers


def _same_file(mod, path):
    f = getattr(mod, "__file__", None)
    try:
        return bool(f) and os.path.samefile(f, path)
    except OSError:                     # pragma: no cover - path vanished
        return False


def _load(key, path):
    """Load ``path`` under ``sys.modules[key]``, reusing an existing entry that
    is already the same file."""
    existing = sys.modules.get(key)
    if existing is not None and _same_file(existing, path):
        return existing
    spec_obj = importlib.util.spec_from_file_location(key, path)
    if spec_obj is None or spec_obj.loader is None:      # pragma: no cover
        raise ImportError("cannot load %s from %s" % (key, path))
    mod = importlib.util.module_from_spec(spec_obj)
    sys.modules[key] = mod
    spec_obj.loader.exec_module(mod)
    return mod


def sibling(name):
    """Import ``<name>.py`` from THIS arm's directory.

    By path, and under ``ladder0_mujoco_<name>``, so the module can neither
    shadow nor be shadowed by a same-named module in another arm.  Use this
    instead of ``import scenes`` / ``import run``.
    """
    return _load(ARM_PREFIX + name, os.path.join(HERE, name + ".py"))


spec = None
SPEC_MODULE = None
_tried = []
for _name in CONTRACT_NAMES:
    _p = os.path.join(LADDER0, _name + ".py")
    _tried.append(_p)
    if os.path.isfile(_p):
        spec = _load(_name, _p)
        SPEC_MODULE = _name
        break

if spec is None:                        # pragma: no cover - env dependent
    raise ImportError(
        "ladder0 shared contract not found (tried %s).  The MuJoCo arm will "
        "not substitute its own constants." % ", ".join(_tried))

# Alias under every name the contract has gone by, so `analysis` resolves it
# whichever one it imports.
for _name in CONTRACT_NAMES:
    sys.modules.setdefault(_name, spec)

_ap = os.path.join(LADDER0, "analysis.py")
if os.path.isfile(_ap):
    analysis = _load("ladder0_analysis", _ap)
    ANALYSIS_AVAILABLE = True
    ANALYSIS_ERROR = None
else:                                   # pragma: no cover - env dependent
    analysis = None
    ANALYSIS_AVAILABLE = False
    ANALYSIS_ERROR = "not found at %s" % _ap
