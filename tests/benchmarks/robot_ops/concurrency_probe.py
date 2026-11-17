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
"""N concurrent agents, each sending one request every ~5 s, through the arms' transport."""
import statistics, sys, threading, time
sys.path.insert(0, "O:/omnisim")
from omnisim.ops_bench.competitors import ModelClient
from omnisim.ops_bench.health import PROBE_SYSTEM, PROBE_MESSAGES

key = open("O:/omnilink-keys/omni_key.txt", encoding="utf-8").read().strip()
N, PER = int(sys.argv[1]), int(sys.argv[2])
lat, fails, attempts = [], [], []
lock = threading.Lock()

def agent(i):
    recs = []
    c = ModelClient(key, "g1-engine", "gemini-3.5-flash", "OmniSim-husky", PER * 3, log=recs)
    time.sleep(i * 5.0 / N)          # spread starts over one interval
    for _ in range(PER):
        t0 = time.monotonic()
        try:
            c.complete(PROBE_SYSTEM, PROBE_MESSAGES)
            with lock: lat.append(time.monotonic() - t0)
        except Exception as exc:
            with lock: fails.append(str(exc)[:60])
        time.sleep(5)
    with lock: attempts.extend(r.get("http") for r in recs)

ts = [threading.Thread(target=agent, args=(i,)) for i in range(N)]
[t.start() for t in ts]; [t.join() for t in ts]
lat.sort()
print(f"N={N} answered {len(lat)}/{N*PER} median {statistics.median(lat):.1f}s p90 {lat[int(.9*len(lat))-1]:.1f}s "
      f"max {lat[-1]:.1f}s | attempts {len(attempts)} http429 {attempts.count(429)} other-non200 "
      f"{sum(1 for a in attempts if a not in (200, 429))} | failed {len(fails)} {set(fails)}")
