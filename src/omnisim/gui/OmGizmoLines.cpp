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

#include "OmGizmoLines.hpp"

#include "OmAbstractPose.hpp"
#include "OmLog.hpp"
#include "OmSelection.hpp"
#include "OmTranslateRotateManipulator.hpp"

#include <QtCore/QByteArray>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QList>
#include <QtCore/QString>

#include <cmath>
#include <cstdlib>
#include <map>
#include <utility>

namespace {

  // ---- a minimal OBJ reader for the two handle meshes ----------------------------------------
  // Deliberately not routed through WREN's importer: this must keep working when src/wren is
  // deleted, and the two files are trivial (v + f a//n triangles, Blender export).
  struct HandleMesh {
    bool loaded = false;
    std::vector<float> verts;             // xyz triples
    std::vector<unsigned int> tris;       // 3 indices per triangle
    std::vector<std::pair<int, int>> edges;      // unique undirected edges
    std::vector<std::pair<int, int>> edgeFaces;  // the (up to 2) triangles on each edge; -1 = none
  };

  bool loadHandleMesh(const QString &path, HandleMesh &m) {
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
      return false;
    while (!f.atEnd()) {
      const QByteArray line = f.readLine().trimmed();
      if (line.startsWith("v ")) {
        const QList<QByteArray> t = line.split(' ');
        int got = 0;
        float xyz[3] = {0, 0, 0};
        for (int i = 1; i < t.size() && got < 3; ++i) {
          if (t.at(i).isEmpty())
            continue;
          xyz[got++] = t.at(i).toFloat();
        }
        if (got == 3) {
          m.verts.push_back(xyz[0]);
          m.verts.push_back(xyz[1]);
          m.verts.push_back(xyz[2]);
        }
      } else if (line.startsWith("f ")) {
        const QList<QByteArray> t = line.split(' ');
        std::vector<unsigned int> idx;
        for (int i = 1; i < t.size(); ++i) {
          if (t.at(i).isEmpty())
            continue;
          const QByteArray v = t.at(i).split('/').at(0);
          if (v.isEmpty())
            continue;
          const long n = v.toLong();
          if (n > 0)
            idx.push_back(static_cast<unsigned int>(n - 1));  // OBJ is 1-based
        }
        // Fan-triangulate (the two shipped files are already triangles).
        for (size_t i = 2; i < idx.size(); ++i) {
          m.tris.push_back(idx[0]);
          m.tris.push_back(idx[i - 1]);
          m.tris.push_back(idx[i]);
        }
      }
    }
    f.close();
    if (m.verts.empty() || m.tris.empty())
      return false;
    // Edge -> adjacent-face table, so a silhouette is one facing comparison per edge.
    std::map<std::pair<int, int>, std::pair<int, int>> table;
    const size_t nTris = m.tris.size() / 3;
    for (size_t t = 0; t < nTris; ++t) {
      for (int e = 0; e < 3; ++e) {
        int a = static_cast<int>(m.tris[t * 3 + e]);
        int b = static_cast<int>(m.tris[t * 3 + ((e + 1) % 3)]);
        if (a > b) {
          const int s = a;
          a = b;
          b = s;
        }
        const std::pair<int, int> key(a, b);
        std::map<std::pair<int, int>, std::pair<int, int>>::iterator it = table.find(key);
        if (it == table.end())
          table.insert(std::make_pair(key, std::make_pair(static_cast<int>(t), -1)));
        else if (it->second.second < 0)
          it->second.second = static_cast<int>(t);
      }
    }
    m.edges.reserve(table.size());
    m.edgeFaces.reserve(table.size());
    for (std::map<std::pair<int, int>, std::pair<int, int>>::const_iterator it = table.begin();
         it != table.end(); ++it) {
      m.edges.push_back(it->first);
      m.edgeFaces.push_back(it->second);
    }
    m.loaded = true;
    return true;
  }

  const HandleMesh &arrowMesh() {
    static HandleMesh m;
    static bool tried = false;
    if (!tried) {
      tried = true;
      // "gl:" is the search path OmView3D/OmGuiApplication register onto resources/wren.
      if (!loadHandleMesh(QFileInfo(QStringLiteral("gl:meshes/arrow.obj")).absoluteFilePath(), m))
        OmLog::info("[OmGizmoLines] could not read gl:meshes/arrow.obj -- the translate gizmo "
                    "will not be drawn (its hit test is unaffected).");
    }
    return m;
  }

  const HandleMesh &circularArrowMesh() {
    static HandleMesh m;
    static bool tried = false;
    if (!tried) {
      tried = true;
      if (!loadHandleMesh(QFileInfo(QStringLiteral("gl:meshes/circular_arrow.obj")).absoluteFilePath(), m))
        OmLog::info("[OmGizmoLines] could not read gl:meshes/circular_arrow.obj -- the rotate "
                    "gizmo will not be drawn (its hit test is unaffected).");
    }
    return m;
  }

  // m16 is column-major; transform (x,y,z,1).
  inline void xform(const float *m16, const float *p, float *out) {
    out[0] = m16[0] * p[0] + m16[4] * p[1] + m16[8] * p[2] + m16[12];
    out[1] = m16[1] * p[0] + m16[5] * p[1] + m16[9] * p[2] + m16[13];
    out[2] = m16[2] * p[0] + m16[6] * p[1] + m16[10] * p[2] + m16[14];
  }

  inline void pushVert(std::vector<float> &v, const float *p) {
    v.push_back(p[0]);
    v.push_back(p[1]);
    v.push_back(p[2]);
    v.push_back(0.0f);
    v.push_back(0.0f);
    v.push_back(0.0f);
    v.push_back(0.0f);
    v.push_back(0.0f);
  }

  // handles.vert's scalar, reproduced term for term:
  //   w  = screenScale
  //   w *= dot(vec4(-view[0][2], -view[1][2], -view[2][2], -view[3][2]), modelTransform[3])
  //   w *= 1 / min(projection[0][0], projection[1][1])
  // The middle term is the camera-space depth of the handle's origin. Column-major flat arrays:
  // view[col][row] == view16[col * 4 + row].
  double handleScale(float screenScale, const float *view16, const float *model16, double projMin) {
    const double depth = -(static_cast<double>(view16[2]) * model16[12] +
                           static_cast<double>(view16[6]) * model16[13] +
                           static_cast<double>(view16[10]) * model16[14] +
                           static_cast<double>(view16[14]) * model16[15]);
    if (projMin <= 1e-9)
      return 0.0;
    return static_cast<double>(screenScale) * depth / projMin;
  }

  // Emit the silhouette of `mesh` under `model16` * scale(w): every edge whose two adjacent
  // triangles disagree about facing the eye, plus every boundary edge. That outline is the
  // boundary of exactly the triangle set the picking pass rasterises.
  void emitSilhouette(const HandleMesh &mesh, const float *model16, double w, const float *eye3,
                      std::vector<float> &out, std::vector<float> *trisOut) {
    if (!mesh.loaded || w <= 0.0)
      return;
    const size_t nV = mesh.verts.size() / 3;
    std::vector<float> world(nV * 3);
    for (size_t i = 0; i < nV; ++i) {
      const float local[3] = {static_cast<float>(mesh.verts[i * 3 + 0] * w),
                              static_cast<float>(mesh.verts[i * 3 + 1] * w),
                              static_cast<float>(mesh.verts[i * 3 + 2] * w)};
      xform(model16, local, &world[i * 3]);
    }
    const size_t nT = mesh.tris.size() / 3;
    std::vector<unsigned char> front(nT, 0);
    for (size_t t = 0; t < nT; ++t) {
      const float *a = &world[mesh.tris[t * 3 + 0] * 3];
      const float *b = &world[mesh.tris[t * 3 + 1] * 3];
      const float *c = &world[mesh.tris[t * 3 + 2] * 3];
      const float e1[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
      const float e2[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
      const float n[3] = {e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2],
                          e1[0] * e2[1] - e1[1] * e2[0]};
      const float v[3] = {a[0] - eye3[0], a[1] - eye3[1], a[2] - eye3[2]};
      front[t] = (n[0] * v[0] + n[1] * v[1] + n[2] * v[2]) < 0.0f ? 1 : 0;
      if (trisOut) {
        for (int k = 0; k < 3; ++k) {
          const float *p = &world[mesh.tris[t * 3 + k] * 3];
          trisOut->push_back(p[0]);
          trisOut->push_back(p[1]);
          trisOut->push_back(p[2]);
        }
      }
    }
    for (size_t e = 0; e < mesh.edges.size(); ++e) {
      const int f0 = mesh.edgeFaces[e].first, f1 = mesh.edgeFaces[e].second;
      const bool silhouette =
        f1 < 0 || (front[static_cast<size_t>(f0)] != front[static_cast<size_t>(f1)]);
      if (!silhouette)
        continue;
      pushVert(out, &world[mesh.edges[e].first * 3]);
      pushVert(out, &world[mesh.edges[e].second * 3]);
    }
  }

  OmTranslateRotateManipulator *liveManipulator() {
    OmSelection *const sel = OmSelection::instance();
    if (!sel)
      return NULL;
    OmAbstractPose *const pose = sel->selectedAbstractPose();
    if (!pose)
      return NULL;
    OmTranslateRotateManipulator *const m = pose->translateRotateManipulator();
    if (!m || !m->isAttached())
      return NULL;
    return m;
  }

}  // namespace

namespace OmGizmoLines {

  bool anyVisible() {
    return liveManipulator() != NULL;
  }

  double debugHandleScale(const float view16[16], double projMin) {
    OmTranslateRotateManipulator *const m = liveManipulator();
    if (!m || !view16)
      return 0.0;
    float mm[16];
    if (!m->axesMatrix(mm) && !m->translationHandleMatrix(0, mm))
      return 0.0;
    return handleScale(m->handleScreenScale(), view16, mm, projMin);
  }

  void collect(const float view16[16], double projMin, const float eye3[3],
               std::vector<float> &outX, std::vector<float> &outY, std::vector<float> &outZ,
               std::vector<Handle> *handlesOut) {
    outX.clear();
    outY.clear();
    outZ.clear();
    if (handlesOut)
      handlesOut->clear();
    OmTranslateRotateManipulator *const m = liveManipulator();
    if (!m || !view16)
      return;
    const float screenScale = m->handleScreenScale();
    std::vector<float> *const dst[3] = {&outX, &outY, &outZ};

    // The three unit axis lines (WREN drew them off the manipulator's own transform: 0 -> +axis,
    // one per colour). Decoration in WREN too -- not pickable there, not claimed pickable here.
    {
      float am[16];
      if (m->axesMatrix(am)) {
        const double w = handleScale(screenScale, view16, am, projMin);
        if (w > 0.0) {
          for (int a = 0; a < 3; ++a) {
            float o[3], tip[3];
            const float zero[3] = {0.0f, 0.0f, 0.0f};
            float unit[3] = {0.0f, 0.0f, 0.0f};
            unit[a] = static_cast<float>(w);
            xform(am, zero, o);
            xform(am, unit, tip);
            pushVert(*dst[a], o);
            pushVert(*dst[a], tip);
          }
        }
      }
    }

    for (int a = 0; a < 3; ++a) {
      float mm[16];
      // Per-handle visibility is WREN parity: during a drag highlightAxis() hides every other
      // handle, and the hit test (OmScenePicker renders exactly these triangles) follows.
      if (m->isTranslationHandleVisible(a) && m->translationHandleMatrix(a, mm)) {
        const double w = handleScale(screenScale, view16, mm, projMin);
        Handle h;
        h.axis = a;
        h.rotate = false;
        emitSilhouette(arrowMesh(), mm, w, eye3, *dst[a], handlesOut ? &h.tris : NULL);
        if (handlesOut && !h.tris.empty())
          handlesOut->push_back(h);
      }
      if (m->isRotationHandleVisible(a) && m->rotationHandleMatrix(a, mm)) {
        const double w = handleScale(screenScale, view16, mm, projMin);
        Handle h;
        h.axis = a;
        h.rotate = true;
        emitSilhouette(circularArrowMesh(), mm, w, eye3, *dst[a], handlesOut ? &h.tris : NULL);
        if (handlesOut && !h.tris.empty())
          handlesOut->push_back(h);
      }
    }
  }

}  // namespace OmGizmoLines
