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

#ifndef OM_MOTION_HISTORY_HPP
#define OM_MOTION_HISTORY_HPP

#include "OmWgpuRenderTarget.hpp"
#include <algorithm>
#include <cstring>
#include <map>
#include <utility>

// CPU half of per-render-target motion tracking. No engine or graphics API calls.
class OmMotionHistory {
  struct Previous {
    std::array<float, 16> model;
    uint64_t revision = 0;
    uint64_t frame = 0;
    std::vector<float> positions;
    std::vector<uint32_t> indices;
  };
  using Key = std::pair<uint64_t, uint32_t>;
  std::map<Key, Previous> mPrevious;
  uint64_t mFrame = 0;

public:
  struct Slot {
    float model[16], previous[16], uvA[4], uvB[4], params[4];
  };
  static_assert(sizeof(Slot) == 176, "WGSL MotionSlot stride");
  std::vector<Slot> records;
  std::vector<uint32_t> draws;
  std::vector<std::array<float, 4>> positions;

  void reset() { mPrevious.clear(); }

  template<class Visible>
  void prepare(const OmWgpuSolidDraw *input, uint32_t count, bool historyValid, Visible visible) {
    if (!historyValid) reset();
    records.clear(); draws.clear(); positions.clear();
    ++mFrame;
    for (uint32_t i = 0; i < count; ++i) {
      const auto &d = input[i];
      if (!d.vertexBuffer || !d.indexBuffer || !d.indexCount || !d.modelMatrix16 || !visible(d)) continue;
      if (d.translucent && d.baseColorA <= 0) continue;
      const Key key(d.motionId, d.motionPart);
      auto found = mPrevious.find(key);
      Previous *old = found == mPrevious.end() ? nullptr : &found->second;
      bool valid = d.motionId && d.geometryRevision && old && old->frame + 1 == mFrame && !d.translucent;
      bool deformed = false;
      if (valid && d.deforming) {
        // An unchanged index count is insufficient: topology edits can reorder
        // vertices. Compare the full correspondence before using old positions.
        valid = d.cpuPositions && d.cpuIndices && !old->positions.empty() &&
                old->positions.size() == d.cpuPositions->size() && old->indices == *d.cpuIndices;
        deformed = valid && old->positions != *d.cpuPositions;
      } else if (valid) valid = old->revision == d.geometryRevision;
      const bool moved = valid && std::memcmp(old->model.data(), d.modelMatrix16, 64) != 0;
      if (historyValid && (!valid || moved || deformed)) {
        Slot slot = {};
        std::memcpy(slot.model, d.modelMatrix16, 64);
        std::memcpy(slot.previous, valid ? old->model.data() : d.modelMatrix16, 64);
        std::memcpy(slot.uvA, d.uvA, 16);
        std::memcpy(slot.uvB, d.uvB, 8);
        slot.params[3] = d.translucent ? 1.0f : 0.0f;
        if (deformed) {
          // Bound the additional storage below WebGPU's default 128 MiB limit.
          // Oversize meshes reject history instead of reading a truncated stream.
          if (positions.size() + old->positions.size()/3 > 4*1024*1024) valid = false;
          else {
            slot.params[1] = 1;
            slot.params[2] = static_cast<float>(positions.size());
            for (size_t p = 0; p+2 < old->positions.size(); p += 3)
              positions.push_back({old->positions[p], old->positions[p+1], old->positions[p+2], 1});
          }
        }
        slot.params[0] = valid ? 1.0f : -1.0f;
        records.push_back(slot);
        draws.push_back(i);
      }
      if (d.motionId && !d.translucent) {
        // Retain the map entry and vector capacity across static frames.
        Previous &current = mPrevious[key];
        std::memcpy(current.model.data(), d.modelMatrix16, 64);
        current.revision = d.geometryRevision;
        current.frame = mFrame;
        if (d.deforming && d.cpuPositions && d.cpuIndices) {
          current.positions = *d.cpuPositions;
          current.indices = *d.cpuIndices;
        } else { current.positions.clear(); current.indices.clear(); }
      }
    }
    // Removed/hidden objects cannot inherit history on reappearance. A failed
    // submission invalidates the target's TAA flag, clearing this map next time.
    for (auto it = mPrevious.begin(); it != mPrevious.end(); ) {
      if (it->second.frame != mFrame) it = mPrevious.erase(it);
      else ++it;
    }
  }
};
#endif
