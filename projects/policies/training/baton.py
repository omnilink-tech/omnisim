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

"""Compatibility import for BATON's historical project path.

The implementation is now part of the installable :mod:`omnisim.policy` API.  This
module deliberately remains so existing controllers, deployment scripts, and saved
G1 demo recipes continue importing the same names without changing their behaviour.

⚠️ Controllers run under the ENGINE with ``import omnisim`` bound to the controller
runtime shim (``lib/controller/python/omnisim`` -- Robot/Supervisor only, which has
NO ``policy`` subpackage).  In that context ``from omnisim.policy.baton import *``
raises ``ModuleNotFoundError``, so we fall back to loading the product
implementation directly from the repo tree by file path.  Regression caught by the
go2 BATON deploy dry-run before the v5.3.0 release; the plain import is kept as the
fast case for standalone scripts and tests where ``omnisim`` is the product package.
"""

try:
    from omnisim.policy.baton import *  # noqa: F401,F403
    from omnisim.policy.baton import __all__  # noqa: F401
except ModuleNotFoundError:  # engine controller: `omnisim` == the runtime shim
    import importlib.util as _ilu
    import sys as _sys
    from pathlib import Path as _Path

    _impl = next(
        (_p / "omnisim" / "policy" / "baton.py")
        for _p in _Path(__file__).resolve().parents
        if (_p / "omnisim" / "policy" / "baton.py").is_file()
    )
    # Isolated module name (the real ``omnisim.policy`` is unreachable here) --
    # registered in sys.modules BEFORE exec so @dataclass / postponed annotations
    # resolve ``cls.__module__`` during class creation.
    _spec = _ilu.spec_from_file_location("_omnisim_policy_baton_impl", _impl)
    _mod = _ilu.module_from_spec(_spec)
    _sys.modules[_spec.name] = _mod
    _spec.loader.exec_module(_mod)
    __all__ = list(
        getattr(_mod, "__all__", [n for n in vars(_mod) if not n.startswith("_")])
    )
    globals().update({n: getattr(_mod, n) for n in __all__})
    del _ilu, _sys, _Path, _impl, _spec, _mod
