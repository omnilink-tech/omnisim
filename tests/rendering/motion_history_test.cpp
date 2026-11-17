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
#include "OmMotionHistory.hpp"
#include <cassert>
#include <cstdio>

int main() {
  OmMotionHistory history;
  std::array<float,16> a = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}, b = a;
  OmWgpuSolidDraw draws[2];
  for (int i = 0; i < 2; ++i) {
    draws[i].vertexBuffer = draws[i].indexBuffer = reinterpret_cast<void *>(1);
    draws[i].indexCount = 3; draws[i].geometryRevision = 7; draws[i].motionId = i+1;
  }
  draws[0].modelMatrix16 = a.data(); draws[1].modelMatrix16 = b.data();
  const auto visible = [](const OmWgpuSolidDraw &) { return true; };
  auto prepare = [&](bool valid = true) { history.prepare(draws, 2, valid, visible); };
  prepare(false);
  assert(history.records.empty()); // the whole first frame already rejects history
  prepare(); assert(history.records.empty());  // no extra geometry for static scenes
  a[12] = 2;
  std::swap(draws[0], draws[1]);  // list order must not become object identity
  prepare();
  assert(history.draws.size() == 1 && history.draws[0] == 1);
  assert(history.records[0].previous[12] == 0 && history.records[0].model[12] == 2);
  assert(history.records[0].params[0] == 1);
  draws[1].geometryRevision++;
  prepare(); assert(history.records[0].params[0] == -1);
  prepare(); assert(history.records.empty());
  draws[1].motionId = 3; // replacement even with identical GPU handles and geometry
  prepare(); assert(history.records[0].params[0] == -1);
  history.prepare(draws, 1, true, visible);
  prepare(); assert(history.records[0].params[0] == -1); // disappeared and returned
  prepare(false); assert(history.records.empty()); // resize / failed frame

  std::vector<float> positions = {0,0,0, 1,0,0, 0,1,0};
  std::vector<uint32_t> indices = {0,1,2};
  draws[1].deforming = true; draws[1].cpuPositions = &positions; draws[1].cpuIndices = &indices;
  prepare(); assert(history.records[0].params[0] == -1);
  positions[2] = .4f; draws[1].geometryRevision++;
  prepare();
  assert(history.records.size() == 1 && history.records[0].params[0] == 1 && history.records[0].params[1] == 1);
  assert(history.positions.size() == 3 && history.positions[0][2] == 0); // owns old data
  prepare(); assert(history.records.empty());
  indices = {1,0,2};
  prepare(); assert(history.records[0].params[0] == -1); // same count, different topology
  draws[1].translucent = true;
  prepare(); assert(history.records[0].params[0] == -1 && history.records[0].params[3] == 1);
  draws[1].translucent = false;
  prepare(); assert(history.records[0].params[0] == -1);
  draws[1].motionId = 0;
  prepare(); assert(history.records[0].params[0] == -1); // unknown particle identity
  std::puts("motion history: PASS");
}
