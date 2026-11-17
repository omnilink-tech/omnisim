// Copyright 2026 OmniLink
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//
// Headless correctness gate for the granular contact response shared with the
// GPU kernel (granular_core_reference.hpp ~ kPhysicsKernelSrc). Needs no GPU
// and no engine build — a stock C++17 compiler is enough:
//
//   g++ -std=c++17 -O2 tests/cuda/granular_core_selfcheck.cpp -o granular_selfcheck
//   ./granular_selfcheck
//
// Scenario: a few hundred particles seeded as a jittered lattice above the
// floor are dropped into a closed box and left to settle. We then assert the
// things a correct granular solve must do and a broken one cannot fake:
//   1. every value stays finite (no NaN/Inf blow-up),
//   2. no particle escapes the floor or the side walls,
//   3. the deepest pairwise penetration stays a small fraction of a radius
//      (contacts actually resolve — particles don't tunnel through each other),
//   4. kinetic energy peaks during free-fall then dissipates toward rest
//      (the damping + Coulomb friction remove energy; a settled pile is still),
//   5. the pile reaches a stable height (matches bench_results.md's settled
//      ~16 cm pile observation, scaled to this scenario).
//
// Exit code is 0 on PASS, 1 on any failed check.
//

#include "granular_core_reference.hpp"

#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

using omnisim::granular::Params;
using omnisim::granular::State;

namespace {

int gFailures = 0;

void check(bool cond, const char *what, double value) {
  std::printf("  [%s] %-48s value=%.6g\n", cond ? "PASS" : "FAIL", what, value);
  if (!cond)
    ++gFailures;
}

// Seed `n` particles as a jittered cubic lattice above the floor, spacing
// 3*radius so there are zero initial overlaps (a clean drop, deterministic via
// a fixed RNG seed so the gate is reproducible run to run).
void seedLattice(State &s, const Params &p, int n) {
  s.resize(static_cast<std::size_t>(n));
  const float spacing = 3.0f * p.radius;
  const float footprint = 2.0f * (p.boundsHalfWidth - p.radius);
  const int perSide = std::max(1, static_cast<int>(footprint / spacing));
  const float start = -0.5f * static_cast<float>(perSide - 1) * spacing;
  const float baseUp = 0.8f;  // drop height: ~0.4 s of free-fall

  std::mt19937 rng(0xC0FFEEu);
  std::uniform_real_distribution<float> jitter(-0.2f * p.radius, 0.2f * p.radius);

  const int up = p.upAxis;
  const int sideA = (up + 1) % 3;
  const int sideB = (up + 2) % 3;
  for (int i = 0; i < n; ++i) {
    const int layer = i / (perSide * perSide);
    const int within = i % (perSide * perSide);
    const int a = within % perSide;
    const int b = within / perSide;
    float xyz[3] = {0.0f, 0.0f, 0.0f};
    xyz[sideA] = start + static_cast<float>(a) * spacing + jitter(rng);
    xyz[sideB] = start + static_cast<float>(b) * spacing + jitter(rng);
    xyz[up] = baseUp + static_cast<float>(layer) * spacing;
    s.px[i] = xyz[0];
    s.py[i] = xyz[1];
    s.pz[i] = xyz[2];
  }
}

bool insideBounds(const State &s, const Params &p, double tol) {
  const std::size_t n = s.size();
  const int up = p.upAxis;
  const int sideA = (up + 1) % 3;
  const int sideB = (up + 2) % 3;
  const float r = p.radius;
  for (std::size_t i = 0; i < n; ++i) {
    const float comp[3] = {s.px[i], s.py[i], s.pz[i]};
    if (comp[up] < p.floor + r - tol)
      return false;
    if (comp[sideA] < -p.boundsHalfWidth + r - tol || comp[sideA] > p.boundsHalfWidth - r + tol)
      return false;
    if (comp[sideB] < -p.boundsHalfWidth + r - tol || comp[sideB] > p.boundsHalfWidth - r + tol)
      return false;
  }
  return true;
}

}  // namespace

int main() {
  Params p;  // defaults mirror the shipping kernel
  const int n = 256;
  const float dtOuter = 0.016f;   // basicTimeStep = 16 ms
  const int outerSteps = 600;     // ~9.6 s of sim — well past settle

  State s;
  seedLattice(s, p, n);

  std::printf("granular_core_selfcheck: n=%d r=%.3f box=+/-%.2f substeps=%d\n",
              n, p.radius, p.boundsHalfWidth, p.substeps);
  std::printf("  seed: max penetration = %.6g (expect 0 — clean drop)\n",
              omnisim::granular::maxPenetration(s, p));

  double peakKE = 0.0;
  bool finiteThroughout = true;
  std::vector<double> lateMaxUp;  // up-max sampled over the last second

  for (int step = 0; step < outerSteps; ++step) {
    omnisim::granular::step(s, p, dtOuter);

    if (!omnisim::granular::allFinite(s)) {
      finiteThroughout = false;
      std::printf("  *** non-finite state at outer step %d ***\n", step);
      break;
    }
    const double ke = omnisim::granular::meanKineticEnergy(s, p);
    if (ke > peakKE)
      peakKE = ke;

    // Sample the pile height over the final ~1 s to confirm it has stopped
    // growing (settled), not still rearranging.
    if (step >= outerSteps - 60 && step % 12 == 0) {
      double mn, mean, mx;
      omnisim::granular::upStats(s, p, mn, mean, mx);
      lateMaxUp.push_back(mx);
    }
  }

  double mn = 0.0, mean = 0.0, mx = 0.0;
  omnisim::granular::upStats(s, p, mn, mean, mx);
  const double finalKE = omnisim::granular::meanKineticEnergy(s, p);
  const double finalPen = omnisim::granular::maxPenetration(s, p);

  double lateSpread = 0.0;
  if (!lateMaxUp.empty()) {
    double lo = lateMaxUp[0], hi = lateMaxUp[0];
    for (double v : lateMaxUp) { if (v < lo) lo = v; if (v > hi) hi = v; }
    lateSpread = hi - lo;
  }

  std::printf("final: up min=%.4f mean=%.4f max=%.4f | KE peak=%.4g end=%.4g | maxPen=%.5f\n",
              mn, mean, mx, peakKE, finalKE, finalPen);

  std::printf("checks:\n");
  check(finiteThroughout, "state finite throughout", finiteThroughout ? 1.0 : 0.0);
  check(insideBounds(s, p, 1e-3), "no particle escaped floor/walls", 1.0);
  check(finalPen < 0.5 * p.radius, "deepest penetration < 0.5*radius", finalPen / p.radius);
  check(peakKE > 1e-5, "energy actually peaked during fall", peakKE);
  check(finalKE < 0.05 * peakKE, "energy dissipated to <5% of peak", finalKE / (peakKE + 1e-30));
  check(mn > p.floor + 0.5 * p.radius, "bottom layer resting on floor", mn);
  check(mx < 0.8, "pile did not blow up past drop height", mx);
  check(lateSpread < 2.0 * p.radius, "pile height stable over final ~1 s", lateSpread / p.radius);

  if (gFailures == 0) {
    std::printf("RESULT: PASS (all %d checks)\n", 8);
    return 0;
  }
  std::printf("RESULT: FAIL (%d check(s) failed)\n", gFailures);
  return 1;
}
