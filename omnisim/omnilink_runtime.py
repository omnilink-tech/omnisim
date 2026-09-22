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
# limitations under the License."""Check the real controller interpreter without keys, network or user-site leakage."""
from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess


def probe(python_exe: str, root: Path) -> dict:
    root = Path(root)
    source = root / "packages" / "omnisim-bridges" / "src"
    paths = [source]
    bundle_site = Path(python_exe).parent / "site-packages"
    code = "\n".join([
        "import sys, json, inspect, importlib.metadata as metadata",
        f"sys.path[:0] = {list(map(str, paths))!r}",
        "import requests, truststore, websocket",
        "from PIL import Image",
        "from omnilink.client import OmniLinkClient, OmniLinkAPIError",
        "from omnilink.usage_meter import UsageMeter",
        "from omnisim_bridges import relay",
        "from omnisim_bridges.access import connection_error",
        "version = relay.check_omnilink_installation()",
        "assert relay.OmniLinkClient is not None, 'bridge SDK import failed'",
        "params = inspect.signature(OmniLinkClient.chat).parameters",
        "assert {'engine', 'system_instruction'} <= set(params), 'incompatible SDK chat API'",
        "assert all(callable(getattr(OmniLinkClient, name, None)) for name in ('get_memory', 'set_memory', 'get_usage')), 'incompatible SDK session API'",
        "Image.new('RGB', (2, 2)).tobytes()",
        "print(json.dumps({'version': version, 'sdk_path': inspect.getfile(OmniLinkClient), 'requests': metadata.version('requests')}))",
    ])
    env = {k: v for k, v in os.environ.items()
           if k.upper() not in {'PYTHONPATH', 'PYTHONHOME', 'PYTHONUSERBASE'}}
    try:
        result = subprocess.run([str(python_exe), "-I", "-c", code], env=env,
                                capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.SubprocessError) as exc:
        return {"status": "missing", "detail": f"controller probe failed: {type(exc).__name__}",
                "python": str(python_exe)}
    if result.returncode:
        error = (result.stderr or result.stdout).strip().splitlines()
        return {"status": "missing", "detail": error[-1] if error else "controller import failed",
                "python": str(python_exe)}
    try:
        data = json.loads(result.stdout.strip().splitlines()[-1])
    except (ValueError, IndexError):
        return {"status": "missing", "detail": "invalid controller dependency probe result",
                "python": str(python_exe)}
    if bundle_site.is_dir() and not Path(data['sdk_path']).resolve().is_relative_to(bundle_site.resolve()):
        return {"status": "missing", "detail": "SDK resolved outside the controller bundle",
                "python": str(python_exe)}
    return {"status": "ok", "detail": f"SDK {data['version']}, transport and capture imports OK",
            "python": str(python_exe), **data}
