// Copyright 1996-2024 Cyberbotics Ltd.
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
// Modifications copyright 2026 OmniLink, licensed under the Apache License, Version 2.0.

#include "OmTriangleMeshCache.hpp"

#include "OmCoordinate.hpp"
#include "OmMFInt.hpp"
#include "OmNormal.hpp"
#include "OmSFBool.hpp"
#include "OmSFDouble.hpp"
#include "OmTextureCoordinate.hpp"
#include "OmTriangleMesh.hpp"
#include "OmTriangleMeshGeometry.hpp"

#include <cassert>
#include <cstdlib>
#include <functional>
#include <limits>

namespace OmTriangleMeshCache {
  static uint64_t gUseSerial = 0;

  static size_t unusedEntryLimit() {
    const char *value = std::getenv("OMNISIM_TRIANGLE_MESH_CACHE_SIZE");
    if (!value || !*value)
      return 128;

    char *end = NULL;
    const unsigned long parsed = std::strtoul(value, &end, 10);
    return end != value && *end == '\0' ? static_cast<size_t>(parsed) : 128;
  }

  static void pruneUnused(OmTriangleMeshMap &map) {
    size_t unusedCount = 0;
    for (const auto &entry : map) {
      if (entry.second.mNumUsers == 0)
        ++unusedCount;
    }

    const size_t limit = unusedEntryLimit();
    while (unusedCount > limit) {
      auto oldest = map.end();
      uint64_t oldestUse = std::numeric_limits<uint64_t>::max();
      for (auto it = map.begin(); it != map.end(); ++it) {
        if (it->second.mNumUsers == 0 && it->second.mLastUse < oldestUse) {
          oldest = it;
          oldestUse = it->second.mLastUse;
        }
      }
      assert(oldest != map.end());
      delete oldest->second.mTriangleMesh;
      map.erase(oldest);
      --unusedCount;
    }
  }

  const highwayhash::HH_U64 SIPHASH_KEY[2] = {
    0x4242424242424242ull,
    0x4242424242424242ull,
  };

  uint64_t sipHash13c(const char *bytes, const int size) {
    return highwayhash::SipHash13(SIPHASH_KEY, bytes, size);
  }
  TriangleMeshInfo::TriangleMeshInfo() : mTriangleMesh(NULL), mNumUsers(0), mLastUse(0) {
  }
  TriangleMeshInfo::TriangleMeshInfo(OmTriangleMesh *triangleMesh) :
    mTriangleMesh(triangleMesh),
    mNumUsers(1),
    mLastUse(++gUseSerial) {
  }

  TriangleMeshGeometryKey::TriangleMeshGeometryKey() {
    mHash = 0;
  }
  TriangleMeshGeometryKey::TriangleMeshGeometryKey(const OmTriangleMeshGeometry *triangleMeshGeometry) {
    set(triangleMeshGeometry);
  }

  void TriangleMeshGeometryKey::set(const OmTriangleMeshGeometry *triangleMeshGeometry) {
    mHash = triangleMeshGeometry->computeHash();
  }

  bool TriangleMeshGeometryKey::operator==(const TriangleMeshGeometryKey &rhs) const {
    return mHash == rhs.mHash;
  }

  std::size_t TriangleMeshGeometryKeyHasher::operator()(const TriangleMeshGeometryKey &k) const {
    assert(sizeof(size_t) == sizeof(uint64_t));
    return static_cast<size_t>(k.mHash);
  }

  void useTriangleMesh(OmTriangleMeshGeometry *user) {
    OmTriangleMeshMap &map = user->getTriangleMeshMap();
    auto it = map.find(user->getMeshKey());
    if (it == map.end() || (!it->second.mTriangleMesh->isValid() && it->second.mNumUsers == 0)) {
      if (it != map.end()) {
        delete it->second.mTriangleMesh;
        map.erase(it);
      }
      map[user->getMeshKey()] = user->createTriangleMesh();
    } else {
      ++it->second.mNumUsers;
      it->second.mLastUse = ++gUseSerial;
    }

    user->setTriangleMesh(map.at(user->getMeshKey()).mTriangleMesh);
    user->updateOdeData();
  }

  void releaseTriangleMesh(OmTriangleMeshGeometry *user) {
    if (user->getTriangleMeshMap().find(user->getMeshKey()) == user->getTriangleMeshMap().end())
      return;
    OmTriangleMeshMap &map = user->getTriangleMeshMap();
    TriangleMeshInfo &triangleMeshInfo = map.at(user->getMeshKey());
    --triangleMeshInfo.mNumUsers;
    assert(triangleMeshInfo.mNumUsers >= 0);
    triangleMeshInfo.mLastUse = ++gUseSerial;
    user->setTriangleMesh(NULL);
    pruneUnused(map);
  }

  void clear() {
    OmTriangleMeshMap &map = OmTriangleMeshGeometry::triangleMeshMap();
    for (auto it = map.begin(); it != map.end();) {
      if (it->second.mNumUsers == 0) {
        delete it->second.mTriangleMesh;
        it = map.erase(it);
      } else
        ++it;
    }
  }
}  // namespace OmTriangleMeshCache
