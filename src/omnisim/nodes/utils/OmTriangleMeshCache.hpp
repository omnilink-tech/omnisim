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

#ifndef OM_TRIANGLE_MESH_CACHE_HPP
#define OM_TRIANGLE_MESH_CACHE_HPP

#include "sip_hash.hpp"

#include <cstdint>

class OmTriangleMeshGeometry;
class OmTriangleMesh;

namespace OmTriangleMeshCache {
  extern const highwayhash::HH_U64 SIPHASH_KEY[2];

  template<class T> uint64_t sipHash13x(const T *bytes, const int size) {
    return highwayhash::SipHash13(SIPHASH_KEY, reinterpret_cast<const char *>(bytes), size * sizeof(T));
  }

  // TriangleMeshInfo is shared by all OmTriangleMeshGeometry instances requiring the same OmTriangleMesh.
  struct TriangleMeshInfo {
    TriangleMeshInfo();
    explicit TriangleMeshInfo(OmTriangleMesh *triangleMesh);

    OmTriangleMesh *mTriangleMesh;
    int mNumUsers;
    uint64_t mLastUse;
  };

  // Key type for an instance of OmTriangleMeshGeometry. Instances can share a OmTriangleMesh if their keys compare equal.
  struct TriangleMeshGeometryKey {
    TriangleMeshGeometryKey();
    explicit TriangleMeshGeometryKey(const OmTriangleMeshGeometry *triangleMeshGeometry);

    void set(const OmTriangleMeshGeometry *triangleMeshGeometry);
    bool operator==(const TriangleMeshGeometryKey &rhs) const;

    uint64_t mHash;
  };

  // Key hashing function required by std::unordered_map
  struct TriangleMeshGeometryKeyHasher {
    std::size_t operator()(const TriangleMeshGeometryKey &k) const;
  };

  void useTriangleMesh(OmTriangleMeshGeometry *user);
  void releaseTriangleMesh(OmTriangleMeshGeometry *user);

  // Drop all entries which aren't referenced by a live scene.  Normally unused
  // entries are retained (up to OMNISIM_TRIANGLE_MESH_CACHE_SIZE, default 128)
  // so a world reload can reuse tessellation results.  This explicit lifecycle
  // hook is called after the final world is destroyed.
  void clear();
}  // namespace OmTriangleMeshCache

#endif
