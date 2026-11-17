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

"""Unit test for the opt-in depth-based contact-damage magnitude (L4).

Standalone — `python test_contact_impulse.py` (no Webots runtime needed).
Pins the three invariants of `damage_tracker._contact_impulse_J`:

  1. Opt-in safety: with depth mode OFF the magnitude is byte-identical to
     the historical `mass*|Δv|` proxy across a sweep of inputs — so every
     shipping demo and the ODE battlebot games are provably unchanged.
  2. Depth mode: with depth available (>0) the magnitude is exactly
     `mass*depth_scale*depth` and `used_depth` is True.
  3. Graceful degradation: with depth mode ON but no penetration witness
     (depth 0 — the default XPBD Newton solver — or None / negative —
     the pre-depth wire) the helper falls back to the proxy, `used_depth`
     False. This is what keeps an XPBD world unaffected even with the flag on.
"""

from __future__ import annotations

import sys

from damage_tracker import _contact_impulse_J

# (mass, dv_mag, depth) sweep covering rest, light, and hard contacts.
CASES = [
    (0.0, 0.0, 0.0),
    (2.637, 0.0, 0.0002),      # resting wheel: tiny penetration, no dv
    (2.637, 0.05, 0.001),      # rolling contact
    (5.0, 1.5, 0.015),         # bumper impact
    (46.0, 3.2, 0.04),         # heavy chassis-slab strike
    (1.0, 10.0, 0.2),          # extreme
]
SCALE = 100.0
FAILS: list[str] = []


def check(name: str, got, want) -> None:
    if got != want:
        FAILS.append(f"{name}: got {got!r}, want {want!r}")


# 1. Opt-in safety — OFF is byte-identical to the proxy, regardless of depth.
for mass, dv, depth in CASES:
    imp, used = _contact_impulse_J(mass, dv, depth, use_depth=False,
                                   depth_scale=SCALE)
    check(f"OFF impulse(mass={mass},dv={dv})", imp, mass * dv)
    check(f"OFF used_depth(mass={mass},dv={dv})", used, False)
    # depth is ignored entirely when off — even a huge penetration mustn't leak.
    imp_big, _ = _contact_impulse_J(mass, dv, 99.0, use_depth=False,
                                    depth_scale=SCALE)
    check(f"OFF ignores depth(mass={mass})", imp_big, mass * dv)

# 2. Depth mode — real penetration scores on mass*scale*depth.
for mass, dv, depth in CASES:
    imp, used = _contact_impulse_J(mass, dv, depth, use_depth=True,
                                   depth_scale=SCALE)
    if depth > 0.0:
        check(f"ON depth(mass={mass},depth={depth})", imp, mass * SCALE * depth)
        check(f"ON used_depth(mass={mass},depth={depth})", used, True)
    else:
        # depth==0 -> graceful fallback to proxy (XPBD).
        check(f"ON depth0 fallback(mass={mass})", imp, mass * dv)
        check(f"ON depth0 used_depth(mass={mass})", used, False)

# 3. Graceful degradation — None / negative depth fall back to the proxy.
for bad in (None, 0.0, -0.001):
    imp, used = _contact_impulse_J(3.0, 2.0, bad, use_depth=True,
                                   depth_scale=SCALE)
    check(f"ON bad-depth={bad} fallback", imp, 3.0 * 2.0)
    check(f"ON bad-depth={bad} used_depth", used, False)

# scale linearity sanity: doubling scale doubles the depth-scored magnitude.
a, _ = _contact_impulse_J(2.0, 0.0, 0.01, use_depth=True, depth_scale=50.0)
b, _ = _contact_impulse_J(2.0, 0.0, 0.01, use_depth=True, depth_scale=100.0)
check("scale linearity", b, 2.0 * a)

if FAILS:
    print("FAIL:")
    for f in FAILS:
        print("  -", f)
    sys.exit(1)
print(f"PASS — {len(CASES)} cases × 3 invariants, opt-in byte-identity held")
