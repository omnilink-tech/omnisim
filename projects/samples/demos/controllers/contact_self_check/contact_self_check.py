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

"""Contact-points self-read check + contact-depth verification gate.

A supervisor reads getContactPoints(True) on its own Solid while resting on a
RectangleArena floor and writes the contact count + per-point penetration depth
(ContactPoint.depth, the ODE dContactGeom.depth exposed 2026-05-29) to
CONTACT_CHECK_LOG. Asserts (a) a non-zero contact count and (b) a populated
numeric depth on a contacting step. This is the regression gate for the
contact-depth wire (physics-contact-impulse-api.md §6).
"""
import os
import sys
import traceback

LOG = os.environ.get(
    "CONTACT_CHECK_LOG",
    os.path.join(os.environ.get("OMNISIM_HOME", "."), "_scratch", "contact_self_check.log"))
f = open(LOG, "w", buffering=1)
try:
    from omnisim import Supervisor
    r = Supervisor()
    step = int(r.getBasicTimeStep())
    self_node = r.getSelf()
    f.write("[check] controller up\n")
    max_contacts = 0
    depth_seen = False  # a finite, non-None depth observed on a contacting step
    for n in range(150):
        if r.step(step) == -1:
            break
        if n % 5:
            continue
        pts = self_node.getContactPoints(True) or []
        if pts:
            max_contacts = max(max_contacts, len(pts))
            d0 = getattr(pts[0], "depth", None)
            if isinstance(d0, (int, float)):
                depth_seen = True
            z = (self_node.getPosition() or [0, 0, 0])[2]
            f.write(f"[check] step={n} bz={z:.3f} contacts={len(pts)} "
                    f"depth0={d0} pt0={list(pts[0].point)}\n")
    ok = max_contacts > 0 and depth_seen
    f.write(f"[check] RESULT max_contacts={max_contacts} depth_populated={depth_seen} "
            f"{'PASS' if ok else 'FAIL'}\n")
except Exception:
    f.write("[check] EXCEPTION:\n" + traceback.format_exc() + "\n")
finally:
    f.flush()
    f.close()
sys.exit(0)
