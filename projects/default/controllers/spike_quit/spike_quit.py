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

"""spike_quit — tiny Supervisor that runs for N steps then calls
Supervisor.simulationQuit(0) so the headless run exits cleanly and any
attached --log-performance file gets flushed.

N is read from `controllerArgs` (first integer). Default 500.

Used by scripts/dev/granular_spike.py.
"""

from __future__ import annotations

import sys

from omnisim import Supervisor


def main() -> int:
    sup = Supervisor()
    ts = int(sup.getBasicTimeStep())
    args = sup.getCustomData() if hasattr(sup, "getCustomData") else ""
    n_steps = 500
    raw = list(sys.argv[1:])
    if raw:
        try:
            n_steps = int(raw[0])
        except ValueError:
            pass
    elif args:
        try:
            n_steps = int(args.strip())
        except ValueError:
            pass
    for _ in range(n_steps):
        if sup.step(ts) == -1:
            break
    sup.simulationQuit(0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
