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

#ifndef OM_ASSIMP_IO_SYSTEM_HPP
#define OM_ASSIMP_IO_SYSTEM_HPP

//
// Description: Assimp file access that can open long Windows paths.
//
// Assimp's default IO system opens files with _wfopen on Windows, which cannot
// open a path longer than MAX_PATH (260 characters) unless the executable is
// long-path aware. This one hands _wfopen the absolute path with the
// "\\?\" long-path prefix, which can. Before this, a mesh in a deep folder (a URDF
// package unpacked a few levels down is enough) failed to import and the robot
// silently lost that geometry. Installing this handler on an Assimp::Importer
// routes every file it opens -- the mesh and any sibling it pulls in (an OBJ's
// .mtl, a glTF's .bin) -- through that path. No Qt: the engine core's Qt-include
// ratchet (tests/test_qt_include_ratchet.py) only goes down.
//

#include <assimp/IOStream.hpp>
#include <assimp/IOSystem.hpp>
#include <assimp/types.h>  // aiMatrix3x3, aiVector3D and their inline operators

struct aiNode;

class OmAssimpIoSystem : public Assimp::IOSystem {
public:
  bool Exists(const char *file) const override;
  char getOsSeparator() const override { return '/'; }
  Assimp::IOStream *Open(const char *file, const char *mode = "rb") override;
  void Close(Assimp::IOStream *stream) override;
};

// The absolute (scene-root) transform of an Assimp node: root * ... * parent * node,
// the order that maps the node's mesh vertices into the scene frame. Shared by
// OmMesh and OmCadShape, which both walk the node graph.
//
// Both used to compose it leaf-first (node * parent * ... * root, inherited from
// Webots), which is the right answer only when at most one level carries a
// transform. Collada always has two: the importer's own up-axis rotation on the
// root, and any node translation that aiProcess_OptimizeGraph cannot bake away
// (it keeps the node of every mesh that is instanced more than once). Found
// 2026-10-15 on Rover Robotics' MEGA: mega_wheels.dae came out 26.5 mm off its
// axle (the base bounced +/-2.7 cm and the wheels stalled), mega_payload.dae
// up to 0.96 m out of place. OMNISIM_MESH_NODE_TRANSFORM_FIX=0 (value-parsed)
// restores the leaf-first product.
aiMatrix4x4 omAssimpNodeWorldTransform(const aiNode *node);

// The matrix that maps a mesh's NORMALS into the frame its vertices `transform`
// maps them to: the inverse-transpose of the upper-left 3x3, with no translation.
// Computed as the cofactor matrix (det * inverse-transpose) times sign(det), so it
// needs no division -- a degenerate transform (a zero scale) yields no NaN -- and a
// mirroring transform still maps an outward normal outward. Its length is not
// preserved: pass each normal through omAssimpTransformNormal(), which renormalises.
//
// Both node walks used to map normals with the full 4x4 (translation included, and
// no inverse-transpose), so a mesh on a translated node -- every Collada geometry
// instanced more than once keeps its node, see above -- shaded with normals bent
// toward the node's offset, and a non-uniform node scale tilted them off the
// surface. Visual only: colliders never read normals. Noted 2026-10-15 with the
// placement fix, fixed 2026-10-23. OMNISIM_MESH_NORMAL_MATRIX_FIX=0 (value-parsed)
// restores the full-4x4 product (omAssimpNormalMatrixFixEnabled()).
inline aiMatrix3x3 omAssimpNormalMatrix(const aiMatrix4x4 &transform) {
  const aiMatrix3x3 m(transform);
  aiMatrix3x3 c;  // cofactor matrix of m
  c.a1 = m.b2 * m.c3 - m.b3 * m.c2;
  c.a2 = m.b3 * m.c1 - m.b1 * m.c3;
  c.a3 = m.b1 * m.c2 - m.b2 * m.c1;
  c.b1 = m.a3 * m.c2 - m.a2 * m.c3;
  c.b2 = m.a1 * m.c3 - m.a3 * m.c1;
  c.b3 = m.a2 * m.c1 - m.a1 * m.c2;
  c.c1 = m.a2 * m.b3 - m.a3 * m.b2;
  c.c2 = m.a3 * m.b1 - m.a1 * m.b3;
  c.c3 = m.a1 * m.b2 - m.a2 * m.b1;
  if (m.a1 * c.a1 + m.a2 * c.a2 + m.a3 * c.a3 < 0) {  // det(m) < 0: a mirror
    for (unsigned int i = 0; i < 3; ++i)
      for (unsigned int j = 0; j < 3; ++j)
        c[i][j] = -c[i][j];
  }
  return c;
}

// normalMatrix * normal, renormalised (returned as is when it collapses to zero).
inline aiVector3D omAssimpTransformNormal(const aiMatrix3x3 &normalMatrix, const aiVector3D &normal) {
  aiVector3D n = normalMatrix * normal;
  const ai_real length = n.Length();
  if (length > 0)
    n /= length;
  return n;
}

// OMNISIM_MESH_NORMAL_MATRIX_FIX: true (default) maps normals with
// omAssimpNormalMatrix(); =0 restores the full-4x4 product. Value-parsed.
bool omAssimpNormalMatrixFixEnabled();

#endif
