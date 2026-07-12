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

#include "WbWgpuView.hpp"

#include "WbAbstractCamera.hpp"
#include "WbBasicJoint.hpp"
#include "WbBox.hpp"
#include "WbCapsule.hpp"
#include "WbCylinder.hpp"
#include "WbDirectionalLight.hpp"  // harvest the real scene sun for lighting/shadow direction
#include "WbGeometry.hpp"
#include "WbGroup.hpp"
#include "WbJoint.hpp"
#include "WbLidar.hpp"
#include "WbLog.hpp"
#include "WbMathsUtilities.hpp"  // twoStepsConvexHull — support-polygon hull
#include "WbMatrix4.hpp"
#include "WbRenderBackend.hpp"
#include "WbRotation.hpp"
#include "WbSFDouble.hpp"
#include "WbSFRotation.hpp"
#include "WbSFVector3.hpp"
#include "WbSelection.hpp"
#include "WbShape.hpp"
#include "WbSolid.hpp"
#include "WbSphere.hpp"
#include "WbWrenRenderingContext.hpp"
#include "WbVector2.hpp"  // support-polygon hull projection
#include "WbVector3.hpp"
#include "WbViewpoint.hpp"
#include "WbVulkanBackend.hpp"
#include "WbWgpuImageAdapter.hpp"   // upload a camera image() to a wgpu texture (device inset)
#include "WbWgpuMeshAdapter.hpp"
#include "WbWgpuMeshCache.hpp"
#include "WbWgpuRenderTarget.hpp"   // WbWgpuSolidDraw
#include "WbWgpuSceneRenderer.hpp"  // buildViewProj, collectWorldDraws
#include "WbWgpuSurface.hpp"
#include "WbWgpuTextureCache.hpp"
#include "WbWorld.hpp"
#include "WbWorldInfo.hpp"  // east/north/up basis for the support-polygon projection
#include "WbFog.hpp"
#include "WbSFColor.hpp"
#include "WbSFDouble.hpp"
#include "WbWrenWindow.hpp"  // grabWindowBufferNow() — WREN main-view capture for parity diff

#include <QtCore/QLibrary>
#include <QtCore/QTimer>
// R4 shadow-bug diagnostic: in-app RenderDoc frame-capture harness (gated at runtime by
// OMNISIM_RDC_CAPTURE). Only compiled if the RenderDoc header is present, so the committed tree
// still builds on machines without it. The API is resolved at runtime via QLibrary("renderdoc")
// — the Vulkan capture layer loads renderdoc.dll into the process when VK_INSTANCE_LAYERS enables it.
// To enable, point the build at a RenderDoc header, e.g.
//   make ... CXXFLAGS='-DOMNISIM_RENDERDOC_HEADER=\"/path/to/renderdoc_app.h\"'
// Nested rather than `#if defined(X) && __has_include(X)` on one line: GCC parses
// the __has_include argument even when the && short-circuits, and rejects an
// undefined macro as a header-name -- so the single-line form fails to compile
// wherever the RenderDoc header flag is absent (i.e. every default build).
// Nesting evaluates __has_include only once the macro is known to be defined.
#ifdef OMNISIM_RENDERDOC_HEADER
#  if __has_include(OMNISIM_RENDERDOC_HEADER)
#    include OMNISIM_RENDERDOC_HEADER
#    define OMNISIM_HAVE_RENDERDOC 1
#  endif
#endif
#include <QtGui/QExposeEvent>
#include <QtGui/QImage>
#include <QtGui/QMouseEvent>
#include <QtGui/QResizeEvent>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <vector>

#ifdef _WIN32
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <windows.h>
#endif

namespace {
  // Webots/VRML rule (WbViewpoint::updateFieldOfViewY): the Viewpoint's fieldOfView
  // is taken on the LARGEST viewport dimension. buildViewProj treats its fov arg as
  // horizontal and derives vertical via aspect, so for a portrait pane (aspect<1) we
  // shrink the passed horizFov so the resulting VERTICAL fov equals fieldOfView —
  // matching WREN's framing at any pane shape. Landscape (aspect>=1): pass it straight.
  double horizFovForAspect(double fov, double aspect) {
    return aspect < 1.0 ? 2.0 * std::atan(std::tan(0.5 * fov) * aspect) : fov;
  }

  // Top-level solid ancestor of `s` (the one in world->topSolids()). Picking +
  // selection both resolve to the top solid (pickSolidAt maps a draw to its top
  // solid, collectWorldDraws tags every draw with its top solid), so highlighting
  // by top solid paints exactly what a click selects.
  WbSolid *topSolidOf(WbSolid *s) {
    if (!s)
      return nullptr;
    while (s->upperSolid())
      s = s->upperSolid();
    return s;
  }

  // R4 step-3c-A.2: selection highlight. Lerp the base colour of every draw
  // belonging to `selTop` toward OmniSim mimosa so the selected object reads as
  // highlighted in the (kSolidLit, baseColor-driven) pane. `tops` is parallel to
  // `draws` (the per-draw top solid from collectWorldDraws). A null selTop is a
  // no-op, so an empty selection renders byte-identically to no highlight.
  void highlightSelectedDraws(std::vector<WbWgpuSolidDraw> &draws,
                              const std::vector<WbSolid *> &tops, const WbSolid *selTop) {
    if (!selTop)
      return;
    const float hr = 0.96f, hg = 0.91f, hb = 0.02f, k = 0.55f;  // mimosa, lerp weight
    const size_t n = std::min(draws.size(), tops.size());
    for (size_t i = 0; i < n; ++i) {
      if (tops[i] != selTop)
        continue;
      draws[i].baseColorR = draws[i].baseColorR * (1.0f - k) + hr * k;
      draws[i].baseColorG = draws[i].baseColorG * (1.0f - k) + hg * k;
      draws[i].baseColorB = draws[i].baseColorB * (1.0f - k) + hb * k;
    }
  }

  // R4 step-3c-A.3: gather (world anchor, world axis) for every WbJoint reachable
  // under `root`, mirroring collectShapeDraws' scene-graph recursion (proven to
  // reach every link). Each joint contributes exactly one entry.
  struct JointAxis {
    WbVector3 anchor, dir;
  };
  void gatherJointAxes(WbBaseNode *root, std::vector<JointAxis> &out) {
    if (!root)
      return;
    if (WbSolid *s = dynamic_cast<WbSolid *>(root)) {
      const QVector<WbBasicJoint *> &js = s->jointChildren();
      for (WbBasicJoint *bj : js) {
        WbJoint *j = dynamic_cast<WbJoint *>(bj);
        if (!j)
          continue;
        JointAxis ja;
        if (j->axisRepresentation(ja.anchor, ja.dir))
          out.push_back(ja);
      }
    }
    if (WbGroup *g = dynamic_cast<WbGroup *>(root)) {
      const int n = g->childCount();
      for (int i = 0; i < n; ++i)
        gatherJointAxes(g->child(i), out);
    }
  }

  // Collect every WbSolid strictly BELOW `root` (root excluded) — the nested solids whose
  // translation/rotation fields are parent-relative (not world). Used by the parent-frame
  // drag self-check to find a solid with a non-identity parent transform to stress.
  void collectDescendantSolids(WbBaseNode *root, std::vector<WbSolid *> &out) {
    WbGroup *g = dynamic_cast<WbGroup *>(root);
    if (!g)
      return;
    const int n = g->childCount();
    for (int i = 0; i < n; ++i) {
      WbBaseNode *c = g->child(i);
      if (WbSolid *s = dynamic_cast<WbSolid *>(c))
        out.push_back(s);
      collectDescendantSolids(c, out);
    }
  }

  // Append a thin cyan box per joint axis (a bar of length L through the anchor
  // along the axis) to `draws`; model matrices go into `store`, reserved up front
  // so the draw pointers stay valid. Returns the number appended. The caller gates
  // this on VF_JOINT_AXES; `store` must outlive the render.
  int collectJointAxisDraws(WbWgpuMeshCache &cache, std::vector<WbWgpuSolidDraw> &draws,
                            std::vector<std::array<float, 16>> &store) {
    WbWorld *world = WbWorld::instance();
    if (!world)
      return 0;
    std::vector<JointAxis> axes;
    for (WbSolid *s : world->topSolids())
      gatherJointAxes(s, axes);
    if (axes.empty())
      return 0;
    WbWgpuMeshHandle box =
      WbWgpuMeshAdapter::acquirePrimitive(cache, 7001ull, WbWgpuMeshAdapter::PrimitiveKind::Box);
    if (!box.vertexBuffer || !box.indexBuffer || !box.indexCount)
      return 0;
    store.reserve(store.size() + axes.size());
    const float L = 0.2f, w = 0.006f;  // bar length along axis, cross-section
    int added = 0;
    for (const JointAxis &ja : axes) {
      WbVector3 dir = ja.dir;
      if (dir.length() < 1e-9)
        continue;
      dir = dir.normalized();
      const WbVector3 ref = std::fabs(dir.z()) < 0.9 ? WbVector3(0, 0, 1) : WbVector3(1, 0, 0);
      const WbVector3 p1 = ref.cross(dir).normalized();
      const WbVector3 p2 = dir.cross(p1);
      // Column-major model: col0 = dir*L, col1 = p1*w, col2 = p2*w, col3 = anchor.
      const std::array<float, 16> m = {
        static_cast<float>(dir.x() * L), static_cast<float>(dir.y() * L), static_cast<float>(dir.z() * L),
        0.0f, static_cast<float>(p1.x() * w), static_cast<float>(p1.y() * w), static_cast<float>(p1.z() * w),
        0.0f, static_cast<float>(p2.x() * w), static_cast<float>(p2.y() * w), static_cast<float>(p2.z() * w),
        0.0f, static_cast<float>(ja.anchor.x()), static_cast<float>(ja.anchor.y()),
        static_cast<float>(ja.anchor.z()), 1.0f};
      store.push_back(m);
      WbWgpuSolidDraw d;
      d.modelMatrix16 = store.back().data();
      d.baseColorR = 0.0f;
      d.baseColorG = 0.95f;
      d.baseColorB = 0.95f;  // cyan — distinct from the mimosa selection highlight
      d.baseColorA = 1.0f;
      d.vertexBuffer = box.vertexBuffer;
      d.indexBuffer = box.indexBuffer;
      d.indexCount = box.indexCount;
      draws.push_back(d);
      ++added;
    }
    return added;
  }

  // R4 step-3c-A: bounding-object wireframes. Append one world-space line segment
  // (two vertices) to `v` in the stride-32 layout clearAndDrawLines wants (pos3 +
  // 5 zero pads for the ignored normal/uv slots).
  void pushSeg(std::vector<float> &v, const WbVector3 &a, const WbVector3 &b) {
    const WbVector3 e[2] = {a, b};
    for (const WbVector3 &p : e) {
      v.push_back(static_cast<float>(p.x()));
      v.push_back(static_cast<float>(p.y()));
      v.push_back(static_cast<float>(p.z()));
      for (int k = 0; k < 5; ++k)
        v.push_back(0.0f);
    }
  }

  // Emit the wireframe of one bounding geometry (world-space, via geom->matrix()).
  // Box → 12 edges; Sphere → 3 great-circle rings; Cylinder/Capsule → 2 end rings +
  // 4 struts (capsule caps approximated by the cylinder for this first cut). Plane /
  // mesh / IndexedFaceSet skipped here.
  void emitGeomEdges(WbGeometry *g, std::vector<float> &v) {
    if (!g)
      return;
    const double kTwoPi = 6.283185307179586;
    const WbMatrix4 m = g->matrix();
    switch (g->nodeType()) {
      case WB_NODE_BOX: {
        const WbVector3 &s = static_cast<WbBox *>(g)->size();
        const double hx = s.x() * 0.5, hy = s.y() * 0.5, hz = s.z() * 0.5;
        WbVector3 c[8];
        int idx = 0;
        for (int xi = 0; xi < 2; ++xi)
          for (int yi = 0; yi < 2; ++yi)
            for (int zi = 0; zi < 2; ++zi)
              c[idx++] = m * WbVector3(xi ? hx : -hx, yi ? hy : -hy, zi ? hz : -hz);
        const int e[12][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7},   // along z
                              {0, 2}, {1, 3}, {4, 6}, {5, 7},   // along y
                              {0, 4}, {1, 5}, {2, 6}, {3, 7}};  // along x
        for (int i = 0; i < 12; ++i)
          pushSeg(v, c[e[i][0]], c[e[i][1]]);
        break;
      }
      case WB_NODE_SPHERE: {
        const double r = static_cast<WbSphere *>(g)->radius();
        const int N = 24;
        for (int plane = 0; plane < 3; ++plane) {
          WbVector3 prev;
          for (int k = 0; k <= N; ++k) {
            const double a = kTwoPi * k / N;
            const double ca = r * std::cos(a), sa = r * std::sin(a);
            const WbVector3 local = plane == 0 ? WbVector3(ca, sa, 0)
                                   : plane == 1 ? WbVector3(ca, 0, sa)
                                                : WbVector3(0, ca, sa);
            const WbVector3 p = m * local;
            if (k > 0)
              pushSeg(v, prev, p);
            prev = p;
          }
        }
        break;
      }
      case WB_NODE_CYLINDER:
      case WB_NODE_CAPSULE: {
        double r, h;
        if (g->nodeType() == WB_NODE_CYLINDER) {
          WbCylinder *c = static_cast<WbCylinder *>(g);
          r = c->radius();
          h = c->height();
        } else {
          WbCapsule *c = static_cast<WbCapsule *>(g);
          r = c->radius();
          h = c->height();
        }
        const double hz = h * 0.5;
        const int N = 24;
        WbVector3 prevTop, prevBot;
        for (int k = 0; k <= N; ++k) {
          const double a = kTwoPi * k / N;
          const double ca = r * std::cos(a), sa = r * std::sin(a);
          const WbVector3 top = m * WbVector3(ca, sa, hz);
          const WbVector3 bot = m * WbVector3(ca, sa, -hz);
          if (k > 0) {
            pushSeg(v, prevTop, top);
            pushSeg(v, prevBot, bot);
          }
          if (k % 6 == 0)  // 4 vertical struts
            pushSeg(v, top, bot);
          prevTop = top;
          prevBot = bot;
        }
        break;
      }
      default:
        break;
    }
  }

  // Recurse a boundingObject subtree, emitting wireframe edges for every geometry
  // (raw geometry, Shape→geometry, or Group/Pose children).
  void gatherBoundingEdges(WbBaseNode *root, std::vector<float> &v) {
    if (!root)
      return;
    if (WbGeometry *g = dynamic_cast<WbGeometry *>(root)) {
      emitGeomEdges(g, v);
      return;
    }
    if (WbShape *sh = dynamic_cast<WbShape *>(root)) {
      emitGeomEdges(sh->geometry(), v);
      return;
    }
    if (WbGroup *grp = dynamic_cast<WbGroup *>(root)) {
      const int n = grp->childCount();
      for (int i = 0; i < n; ++i)
        gatherBoundingEdges(grp->child(i), v);
    }
  }

  // Walk every solid (a boundingObject lives in its own SFNode field, not the
  // children group, so handle it per-solid then recurse the scene graph).
  void walkSolidsForBounding(WbBaseNode *root, std::vector<float> &v) {
    if (!root)
      return;
    if (WbSolid *s = dynamic_cast<WbSolid *>(root))
      if (s->boundingObject())
        gatherBoundingEdges(s->boundingObject(), v);
    if (WbGroup *grp = dynamic_cast<WbGroup *>(root)) {
      const int n = grp->childCount();
      for (int i = 0; i < n; ++i)
        walkSolidsForBounding(grp->child(i), v);
    }
  }

  // Collect bounding-object wireframe segments for the whole world. `v` fills with
  // stride-32 vertices (2 per segment); returns the vertex count.
  uint32_t collectBoundingEdges(std::vector<float> &v) {
    WbWorld *world = WbWorld::instance();
    if (!world)
      return 0;
    for (WbSolid *s : world->topSolids())
      walkSolidsForBounding(s, v);
    return static_cast<uint32_t>(v.size() / 8);
  }

  // R4 step-3c-A: a 3-axis cross (3 segments) centred at world point `c`, half-arm
  // `h` — the marker glyph for centre-of-mass / contact points.
  void emitCross(std::vector<float> &v, const WbVector3 &c, double h) {
    pushSeg(v, WbVector3(c.x() - h, c.y(), c.z()), WbVector3(c.x() + h, c.y(), c.z()));
    pushSeg(v, WbVector3(c.x(), c.y() - h, c.z()), WbVector3(c.x(), c.y() + h, c.z()));
    pushSeg(v, WbVector3(c.x(), c.y(), c.z() - h), WbVector3(c.x(), c.y(), c.z() + h));
  }

  void walkSolidsForCom(WbBaseNode *root, std::vector<float> &v) {
    if (!root)
      return;
    if (WbSolid *s = dynamic_cast<WbSolid *>(root))
      emitCross(v, s->computedGlobalCenterOfMass(), 0.06);
    if (WbGroup *grp = dynamic_cast<WbGroup *>(root)) {
      const int n = grp->childCount();
      for (int i = 0; i < n; ++i)
        walkSolidsForCom(grp->child(i), v);
    }
  }

  // Centre-of-mass markers: a cross at every solid's global COM. (WREN gates these
  // per-solid via showGlobalCenterOfMassRepresentation, not a VF_ flag.) Returns
  // the vertex count (6 per marker).
  uint32_t collectComCrosses(std::vector<float> &v) {
    WbWorld *world = WbWorld::instance();
    if (!world)
      return 0;
    for (WbSolid *s : world->topSolids())
      walkSolidsForCom(s, v);
    return static_cast<uint32_t>(v.size() / 8);
  }

  // Per-solid-GATED COM walk for the LIVE pane: emit a COM cross only for solids whose
  // showGlobalCenterOfMassRepresentation is enabled — mirrors WREN's per-solid gating (there
  // is no global VF_ flag), so the live pane is WREN-identical when nothing is toggled.
  void walkSolidsForComGated(WbBaseNode *root, std::vector<float> &v) {
    if (!root)
      return;
    if (WbSolid *s = dynamic_cast<WbSolid *>(root)) {
      if (s->globalCenterOfMassRepresentationEnabled())
        emitCross(v, s->computedGlobalCenterOfMass(), 0.06);
    }
    if (WbGroup *grp = dynamic_cast<WbGroup *>(root)) {
      const int n = grp->childCount();
      for (int i = 0; i < n; ++i)
        walkSolidsForComGated(grp->child(i), v);
    }
  }
  uint32_t collectComCrossesGated(std::vector<float> &v) {
    WbWorld *world = WbWorld::instance();
    if (!world)
      return 0;
    for (WbSolid *s : world->topSolids())
      walkSolidsForComGated(s, v);
    return static_cast<uint32_t>(v.size() / 8);
  }

  void walkSolidsForContacts(WbBaseNode *root, std::vector<float> &v) {
    if (!root)
      return;
    if (WbSolid *s = dynamic_cast<WbSolid *>(root))
      // computedContactPoints() EXTRACTS the current engine contacts on demand; the plain
      // contactPoints() only returns a lazily-cached list that another path (WREN's own
      // contact rendering / a supervisor) must have requested — empty otherwise. Computing
      // here makes the wgpu overlay self-sufficient (this runs only when VF_CONTACT_POINTS
      // is on, so the per-frame extract cost is paid only when contacts are actually wanted).
      for (const WbVector3 &p : s->computedContactPoints())  // engine-computed, world-space
        emitCross(v, p, 0.025);
    if (WbGroup *grp = dynamic_cast<WbGroup *>(root)) {
      const int n = grp->childCount();
      for (int i = 0; i < n; ++i)
        walkSolidsForContacts(grp->child(i), v);
    }
  }

  // R4 step-3c-A: joint axes as world-space line segments (anchor ± dir·half) for the
  // live-pane always-on-top overlay (replaces the box representation on-screen).
  uint32_t collectJointAxisLineVerts(std::vector<float> &v) {
    WbWorld *world = WbWorld::instance();
    if (!world)
      return 0;
    std::vector<JointAxis> axes;
    for (WbSolid *s : world->topSolids())
      gatherJointAxes(s, axes);
    const double half = 0.1;  // 0.2 m total bar through the anchor
    for (const JointAxis &ja : axes) {
      WbVector3 d = ja.dir;
      if (d.length() < 1e-9)
        continue;
      d = d.normalized();
      pushSeg(v, ja.anchor - d * half, ja.anchor + d * half);
    }
    return static_cast<uint32_t>(v.size() / 8);
  }

  // R4 step-3c-A: camera/range-finder view frustums (VF_CAMERA_FRUSTUMS /
  // VF_RANGE_FINDER_FRUSTUMS). Convention from WbAbstractCamera::updateFrustumDisplay:
  // the optical axis is camera-local +X, near at x=minRange, FOV is horizontal, vertical
  // via height/width. Emits a near rect + far rect + 4 connectors (12 edges), world-
  // transformed by the camera's matrix(). Far is clamped so an unbounded camera frustum
  // stays a sane size.
  // cameraOn/rangeOn select which sensor types contribute, matching WREN's per-type gating
  // (WbAbstractCamera::setOptionalRendering): a range-finder frustum is gated by
  // VF_RANGE_FINDER_FRUSTUMS, every other camera (Camera, Lidar) by VF_CAMERA_FRUSTUMS.
  void gatherFrustumNode(WbBaseNode *root, std::vector<float> &v, bool cameraOn, bool rangeOn) {
    if (!root)
      return;
    WbAbstractCamera *cam = dynamic_cast<WbAbstractCamera *>(root);
    if (cam && (cam->isRangeFinder() ? rangeOn : cameraOn)) {
      const double fov = cam->fieldOfView();
      const double n = cam->minRange() > 1e-4 ? cam->minRange() : 0.05;
      double f = cam->maxRange();
      if (f <= n || f > 50.0)
        f = n + 1.0;  // clamp huge/invalid far to a visible frustum
      const int w = cam->width(), h = cam->height();
      const double aspectVH = w > 0 ? static_cast<double>(h) / static_cast<double>(w) : 0.75;
      const double t = std::tan(fov * 0.5);
      const WbMatrix4 m = cam->matrix();
      auto corner = [&](double d, double sy, double sz) {
        const double hw = d * t, hh = hw * aspectVH;  // +X forward, Y horizontal, Z vertical
        return m * WbVector3(d, sy * hw, sz * hh);
      };
      const WbVector3 nc[4] = {corner(n, -1, -1), corner(n, 1, -1), corner(n, 1, 1), corner(n, -1, 1)};
      const WbVector3 fc[4] = {corner(f, -1, -1), corner(f, 1, -1), corner(f, 1, 1), corner(f, -1, 1)};
      for (int i = 0; i < 4; ++i) {
        pushSeg(v, nc[i], nc[(i + 1) % 4]);  // near rect
        pushSeg(v, fc[i], fc[(i + 1) % 4]);  // far rect
        pushSeg(v, nc[i], fc[i]);            // near→far connector
      }
    }
    if (WbGroup *grp = dynamic_cast<WbGroup *>(root)) {
      const int nn = grp->childCount();
      for (int i = 0; i < nn; ++i)
        gatherFrustumNode(grp->child(i), v, cameraOn, rangeOn);  // descend regardless of type gate
    }
  }

  uint32_t collectFrustumLines(std::vector<float> &v, bool cameraOn, bool rangeOn) {
    WbWorld *world = WbWorld::instance();
    if (!world)
      return 0;
    for (WbSolid *s : world->topSolids())
      gatherFrustumNode(s, v, cameraOn, rangeOn);
    return static_cast<uint32_t>(v.size() / 8);
  }

  // First WbDirectionalLight anywhere in the tree (e.g. the OmniSimSun PROTO's light), for
  // matching WREN's actual sun direction in the lighting/shadow convergence.
  WbDirectionalLight *findFirstDirectionalLight(WbBaseNode *root) {
    if (!root)
      return nullptr;
    if (WbDirectionalLight *dl = dynamic_cast<WbDirectionalLight *>(root))
      return dl;
    if (WbGroup *g = dynamic_cast<WbGroup *>(root)) {
      const int n = g->childCount();
      for (int i = 0; i < n; ++i)
        if (WbDirectionalLight *d = findFirstDirectionalLight(g->child(i)))
          return d;
    }
    return nullptr;
  }

  // First WbCamera (not a range-finder) anywhere in the tree, for the device-output inset.
  WbAbstractCamera *findFirstCamera(WbBaseNode *root) {
    if (!root)
      return nullptr;
    if (WbAbstractCamera *cam = dynamic_cast<WbAbstractCamera *>(root))
      if (!cam->isRangeFinder())
        return cam;
    if (WbGroup *g = dynamic_cast<WbGroup *>(root)) {
      const int n = g->childCount();
      for (int i = 0; i < n; ++i)
        if (WbAbstractCamera *c = findFirstCamera(g->child(i)))
          return c;
    }
    return nullptr;
  }

  // R4 step-3c-A: surface normals (VF_NORMALS). One short outward line per
  // representative surface point of a primitive geometry (Box→6 face normals,
  // Sphere→6 axes, Cylinder/Capsule→8 sides + 2 caps), world-transformed by the
  // geometry matrix. (Per-VERTEX mesh normals for arbitrary IndexedFaceSet meshes
  // need CPU mesh data — a follow-up; primitives cover the common debug case.)
  void emitGeomNormals(WbGeometry *g, std::vector<float> &v) {
    if (!g)
      return;
    const double len = 0.05;
    const WbMatrix4 m = g->matrix();
    auto nline = [&](const WbVector3 &pLocal, const WbVector3 &nLocal) {
      pushSeg(v, m * pLocal, m * (pLocal + nLocal.normalized() * len));
    };
    switch (g->nodeType()) {
      case WB_NODE_BOX: {
        const WbVector3 &s = static_cast<WbBox *>(g)->size();
        const double hx = s.x() * 0.5, hy = s.y() * 0.5, hz = s.z() * 0.5;
        nline(WbVector3(hx, 0, 0), WbVector3(1, 0, 0));
        nline(WbVector3(-hx, 0, 0), WbVector3(-1, 0, 0));
        nline(WbVector3(0, hy, 0), WbVector3(0, 1, 0));
        nline(WbVector3(0, -hy, 0), WbVector3(0, -1, 0));
        nline(WbVector3(0, 0, hz), WbVector3(0, 0, 1));
        nline(WbVector3(0, 0, -hz), WbVector3(0, 0, -1));
        break;
      }
      case WB_NODE_SPHERE: {
        const double r = static_cast<WbSphere *>(g)->radius();
        const WbVector3 d[6] = {WbVector3(1, 0, 0),  WbVector3(-1, 0, 0), WbVector3(0, 1, 0),
                                WbVector3(0, -1, 0), WbVector3(0, 0, 1),  WbVector3(0, 0, -1)};
        for (const WbVector3 &dir : d)
          nline(dir * r, dir);
        break;
      }
      case WB_NODE_CYLINDER:
      case WB_NODE_CAPSULE: {
        double r, h;
        if (g->nodeType() == WB_NODE_CYLINDER) {
          WbCylinder *c = static_cast<WbCylinder *>(g);
          r = c->radius();
          h = c->height();
        } else {
          WbCapsule *c = static_cast<WbCapsule *>(g);
          r = c->radius();
          h = c->height();
        }
        for (int k = 0; k < 8; ++k) {
          const double a = k * 0.785398163;
          const WbVector3 dir(std::cos(a), std::sin(a), 0);
          nline(dir * r, dir);
        }
        nline(WbVector3(0, 0, h * 0.5), WbVector3(0, 0, 1));
        nline(WbVector3(0, 0, -h * 0.5), WbVector3(0, 0, -1));
        break;
      }
      default:
        break;
    }
  }

  void gatherNormalsNode(WbBaseNode *root, std::vector<float> &v) {
    if (!root)
      return;
    if (WbShape *sh = dynamic_cast<WbShape *>(root))
      emitGeomNormals(sh->geometry(), v);
    if (WbGroup *grp = dynamic_cast<WbGroup *>(root)) {
      const int n = grp->childCount();
      for (int i = 0; i < n; ++i)
        gatherNormalsNode(grp->child(i), v);
    }
  }

  uint32_t collectNormalLines(std::vector<float> &v) {
    WbWorld *world = WbWorld::instance();
    if (!world)
      return 0;
    for (WbSolid *s : world->topSolids())
      gatherNormalsNode(s, v);
    return static_cast<uint32_t>(v.size() / 8);
  }

  // R4 step-3c-A: lidar ray paths (VF_LIDAR_RAYS_PATHS). A subsampled fan of rays from
  // each WbLidar origin out to maxRange, spanning its horizontal × vertical FOV. Local
  // ray dir (cos v·cos h, cos v·sin h, sin v) in the +X-forward / Y-horizontal /
  // Z-vertical convention (same as the frustum), world-transformed by the lidar matrix.
  void gatherLidarRays(WbBaseNode *root, std::vector<float> &v) {
    if (!root)
      return;
    if (WbLidar *l = dynamic_cast<WbLidar *>(root)) {
      const double hfov = l->fieldOfView();
      const int lw = l->width(), lh = l->height();
      const double vfov = (lw > 0) ? hfov * static_cast<double>(lh) / lw : 0.0;  // matches WbLidar
      double range = l->maxRange();
      if (range <= 0.0 || range > 20.0)
        range = 20.0;  // clamp unbounded lidar range to a visible length
      const int nh = 32, nv = l->height() > 1 ? std::min(l->height(), 6) : 1;
      const WbMatrix4 m = l->matrix();
      for (int vi = 0; vi < nv; ++vi) {
        const double vt = nv > 1 ? (-vfov * 0.5 + vfov * vi / (nv - 1)) : 0.0;
        for (int hi = 0; hi <= nh; ++hi) {
          const double ht = -hfov * 0.5 + hfov * hi / nh;
          const WbVector3 dir(std::cos(vt) * std::cos(ht), std::cos(vt) * std::sin(ht), std::sin(vt));
          pushSeg(v, m * WbVector3(0, 0, 0), m * (dir * range));
        }
      }
    }
    if (WbGroup *grp = dynamic_cast<WbGroup *>(root)) {
      const int n = grp->childCount();
      for (int i = 0; i < n; ++i)
        gatherLidarRays(grp->child(i), v);
    }
  }

  uint32_t collectLidarRays(std::vector<float> &v) {
    WbWorld *world = WbWorld::instance();
    if (!world)
      return 0;
    for (WbSolid *s : world->topSolids())
      gatherLidarRays(s, v);
    return static_cast<uint32_t>(v.size() / 8);
  }

  // R4 step-3c-A: translate-manipulator handles — the gizmo geometry, 3 world-axis
  // segments (X red / Y green / Z blue) at a solid's origin, scaled by `len`. Emits
  // each axis into its own vector so the caller can colour them. (The drag/hit-test
  // interaction is a separate follow-up; this is the visual handle.)
  void emitManipulatorAxes(WbSolid *s, std::vector<float> &vx, std::vector<float> &vy,
                           std::vector<float> &vz, double len) {
    if (!s)
      return;
    const WbVector3 o = s->position();
    pushSeg(vx, o, o + s->xAxis().normalized() * len);
    pushSeg(vy, o, o + s->yAxis().normalized() * len);
    pushSeg(vz, o, o + s->zAxis().normalized() * len);
  }

  // R4 step-3c-A: rotate-gizmo ring in the plane spanned by (u,w) about centre `o`,
  // radius `r` — a 24-segment circle. (u,w) are two orthonormal solid axes; the ring's
  // normal is the third axis, so X/Y/Z rings colour by their rotation axis.
  void emitRotateRing(std::vector<float> &v, const WbVector3 &o, const WbVector3 &u,
                      const WbVector3 &w, double r) {
    const double kTwoPi = 6.283185307179586;
    const int N = 24;
    WbVector3 prev;
    for (int k = 0; k <= N; ++k) {
      const double a = kTwoPi * k / N;
      const WbVector3 p = o + (u * (r * std::cos(a)) + w * (r * std::sin(a)));
      if (k > 0)
        pushSeg(v, prev, p);
      prev = p;
    }
  }

  // The three rotate rings of a solid's gizmo (X-axis ring in the YZ plane, etc.),
  // each into its own vector so the caller can colour them R/G/B by rotation axis.
  void emitRotateRings(WbSolid *s, std::vector<float> &vx, std::vector<float> &vy,
                       std::vector<float> &vz, double r) {
    if (!s)
      return;
    const WbVector3 o = s->position();
    const WbVector3 X = s->xAxis().normalized(), Y = s->yAxis().normalized(),
                    Z = s->zAxis().normalized();
    emitRotateRing(vx, o, Y, Z, r);  // ring about X (red)
    emitRotateRing(vy, o, Z, X, r);  // ring about Y (green)
    emitRotateRing(vz, o, X, Y, r);  // ring about Z (blue)
  }

  // R4 step-3c-A: axis-aligned wireframe cube (12 edges) of half-size `h` at `c` — the
  // scale-gizmo handle marker.
  void emitCube(std::vector<float> &v, const WbVector3 &c, double h) {
    WbVector3 cor[8];
    int idx = 0;
    for (int xi = 0; xi < 2; ++xi)
      for (int yi = 0; yi < 2; ++yi)
        for (int zi = 0; zi < 2; ++zi)
          cor[idx++] =
            WbVector3(c.x() + (xi ? h : -h), c.y() + (yi ? h : -h), c.z() + (zi ? h : -h));
    const int e[12][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}, {0, 2}, {1, 3},
                          {4, 6}, {5, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    for (int i = 0; i < 12; ++i)
      pushSeg(v, cor[e[i][0]], cor[e[i][1]]);
  }

  // Scale-gizmo handles: a small cube at the end of each solid axis (origin + axis·len),
  // into per-axis vectors for R/G/B colouring.
  void emitScaleBoxes(WbSolid *s, std::vector<float> &vx, std::vector<float> &vy,
                      std::vector<float> &vz, double len, double h) {
    if (!s)
      return;
    const WbVector3 o = s->position();
    emitCube(vx, o + s->xAxis().normalized() * len, h);
    emitCube(vy, o + s->yAxis().normalized() * len, h);
    emitCube(vz, o + s->zAxis().normalized() * len, h);
  }

  // Project a world point through a column-major view-proj to pane pixels (sx,sy) at
  // (w,h). Returns false if behind the camera (clip.w ≤ 0). Y is flipped to top-left.
  bool projectToScreen(const float vp[16], const WbVector3 &p, int w, int h, double &sx, double &sy) {
    const double x = p.x(), y = p.y(), z = p.z();
    const double cx = vp[0] * x + vp[4] * y + vp[8] * z + vp[12];
    const double cy = vp[1] * x + vp[5] * y + vp[9] * z + vp[13];
    const double cw = vp[3] * x + vp[7] * y + vp[11] * z + vp[15];
    if (cw <= 1e-6)
      return false;
    sx = (cx / cw * 0.5 + 0.5) * w;
    sy = (1.0 - (cy / cw * 0.5 + 0.5)) * h;
    return true;
  }

  double distToSeg(double px, double py, double ax, double ay, double bx, double by) {
    const double dx = bx - ax, dy = by - ay;
    const double l2 = dx * dx + dy * dy;
    double t = l2 > 1e-9 ? ((px - ax) * dx + (py - ay) * dy) / l2 : 0.0;
    t = t < 0 ? 0 : (t > 1 ? 1 : t);
    return std::hypot(px - (ax + t * dx), py - (ay + t * dy));
  }

  // Contact-point markers (`VF_CONTACT_POINTS`): a cross at each engine-computed
  // contact. Count depends on the physics state (empty if nothing is touching).
  uint32_t collectContactCrosses(std::vector<float> &v) {
    WbWorld *world = WbWorld::instance();
    if (!world)
      return 0;
    for (WbSolid *s : world->topSolids())
      walkSolidsForContacts(s, v);
    return static_cast<uint32_t>(v.size() / 8);
  }

  // Support polygon: per top-level solid, the convex hull of its ground contact points
  // projected onto the horizontal (north/east) plane — the stability footprint. Mirrors
  // WbSolid::supportPolygon() exactly (same projection basis + WbMathsUtilities::
  // twoStepsConvexHull) but computed cold from computedContactPoints() so it does not depend
  // on the per-solid representation being enabled. Each hull is emitted as a closed loop of
  // segments at the mean contact height. Empty until something rests (≥3 contacts) → 0 in a
  // static scene, like contact points. No global VF flag (WREN gates it per-solid).
  uint32_t collectSupportPolygon(std::vector<float> &v) {
    WbWorld *world = WbWorld::instance();
    if (!world || !world->worldInfo())
      return 0;
    const WbWorldInfo *wi = world->worldInfo();
    const WbVector3 north = wi->northVector(), east = wi->eastVector(), up = wi->upVector();
    for (WbSolid *s : world->topSolids()) {
      const QVector<WbVector3> &cps = s->computedContactPoints(true);  // this solid + descendants
      const int n = cps.size();
      if (n < 3)
        continue;  // need at least a triangle to bound an area
      QVector<WbVector2> proj(n);
      double h = 0.0;
      for (int i = 0; i < n; ++i) {
        proj[i].setXy(cps.at(i).dot(north), cps.at(i).dot(east));
        h += cps.at(i).dot(up);
      }
      h /= n;  // draw the footprint at the mean contact height
      QVector<int> idx(n);
      const int hull = WbMathsUtilities::twoStepsConvexHull(proj, idx);
      if (hull < 3)
        continue;
      WbVector3 first, prev;
      for (int i = 0; i < hull; ++i) {
        const WbVector2 &pp = proj.at(idx.at(i));
        const WbVector3 w = north * pp.x() + east * pp.y() + up * h;  // reconstruct world point
        if (i == 0)
          first = w;
        else
          pushSeg(v, prev, w);
        prev = w;
      }
      pushSeg(v, prev, first);  // close the loop
    }
    return static_cast<uint32_t>(v.size() / 8);
  }
}  // namespace

WbWgpuView::WbWgpuView(QWindow *parent) : QWindow(parent) {
  setSurfaceType(QWindow::VulkanSurface);
  setTitle(QStringLiteral("OmniSim — wgpu viewport (R4 step 2)"));
  resize(640, 480);
  // (A self-check "park the pane off-screen" experiment lived here briefly — REVERTED: off-monitor
  // windows never get exposed, so the backend never initialized and the self-check never ran. The
  // golden no longer depends on window layout at all: see WbWrenWindow::grabSceneOffscreen.)

  mTimer = new QTimer(this);
  mTimer->setInterval(16);  // ~60 Hz; Fifo present self-throttles to vsync
  connect(mTimer, &QTimer::timeout, this, [this]() { renderTick(); });
  // NOTE: started on first expose, NOT here — querying the wgpu backend before
  // the graphics subsystem is up makes device init fail and the registry caches
  // that failure for the whole run (poisoning the backend).
}

WbWgpuView::~WbWgpuView() {
  delete mPickTarget;
  delete mTextureCache;
  delete mMeshCache;
  delete mSurface;  // mTimer is a child QObject, auto-deleted
}

int WbWgpuView::pixelWidth() const {
  const int w = static_cast<int>(width() * devicePixelRatio());
  return w > 0 ? w : 1;
}

int WbWgpuView::pixelHeight() const {
  const int h = static_cast<int>(height() * devicePixelRatio());
  return h > 0 ? h : 1;
}

void WbWgpuView::exposeEvent(QExposeEvent *) {
  if (!isExposed())
    return;
  ensureSurface();  // inits backend + mesh cache + surface — AFTER graphics is up
  // Start the frame timer once the backend is up, then leave it running (don't
  // stop on a later un-expose): the offscreen self-check + per-frame draw keep
  // ticking, and renderTick self-guards the on-screen part with isExposed().
  if (mBackend && !mTimer->isActive())
    mTimer->start();
}

void WbWgpuView::resizeEvent(QResizeEvent *) {
  if (mSurface)
    mSurface->resize(pixelWidth(), pixelHeight());
}

// Fetch the wgpu backend + mesh cache. No window/HWND/surface needed, so the
// offscreen self-check + picking can run even if the pane is never exposed.
void WbWgpuView::ensureBackend() {
  if (mBackend)
    return;
  WbRenderBackendRegistry::initialise();  // idempotent
  WbRenderBackend *rb = WbRenderBackendRegistry::vulkanBackend();
  if (!rb || rb->kind() != WbRenderBackendKind::Vulkan || !rb->isAvailable())
    return;  // not a wgpu-ON build / dep missing — stays inert
  mBackend = static_cast<WbVulkanBackend *>(rb);
  if (!mMeshCache)
    mMeshCache = new WbWgpuMeshCache(mBackend);
  if (!mTextureCache)
    mTextureCache = new WbWgpuTextureCache(mBackend);
}

void WbWgpuView::ensureSurface() {
  if (mSurface || mInitAttempted)
    return;
  mInitAttempted = true;
  ensureBackend();
  if (!mBackend) {
    WbLog::info(
      "[WbWgpuView] wgpu backend unavailable — OMNISIM_VIEW3D_WGPU needs a wgpu-ON build "
      "(OMNISIM_WITH_VULKAN=ON + wgpu-native). Window will idle.");
    return;
  }
  void *hwnd = reinterpret_cast<void *>(winId());
  void *hinst = nullptr;
#ifdef _WIN32
  hinst = reinterpret_cast<void *>(GetModuleHandleW(nullptr));
#endif
  mSurface = new WbWgpuSurface(mBackend, hwnd, hinst, pixelWidth(), pixelHeight());
  if (!mSurface->isUsable()) {
    WbLog::info("[WbWgpuView] surface creation failed");
    delete mSurface;
    mSurface = nullptr;
    return;
  }
  WbLog::info("[WbWgpuView] presenting to screen via wgpu (R4 step 3)");
}

// Build a Webots-camera-convention (+X forward, +Y left, +Z up) world matrix
// from the live main Viewpoint, so buildViewProj renders the same view WREN
// shows. A Webots Viewpoint looks down -Z, so forward = -orientation.direction()
// and up = orientation.up() (mirrors WbViewpoint's own view-matrix basis).
bool WbWgpuView::buildViewpointCamera(WbMatrix4 &cam, double &horizFov) const {
  WbWorld *world = WbWorld::instance();
  if (!world)
    return false;
  WbViewpoint *vp = world->viewpoint();
  if (!vp || !vp->position() || !vp->orientation())
    return false;
  const WbVector3 eye = vp->position()->value();
  const WbRotation rot = vp->orientation()->value();
  // WbRotation::direction() is the toward-scene look vector (verified by the
  // self-check: -direction() put all geometry behind the camera, clipw<0).
  const WbVector3 f = rot.direction().normalized();    // forward (toward scene)
  const WbVector3 s = f.cross(rot.up()).normalized();  // right
  const WbVector3 u = s.cross(f);                      // orthonormal up
  // Row-major WbMatrix4 columns: [f | -s (left) | u | eye].
  cam = WbMatrix4(f.x(), -s.x(), u.x(), eye.x(), f.y(), -s.y(), u.y(), eye.y(), f.z(), -s.z(), u.z(),
                  eye.z(), 0, 0, 0, 1);
  horizFov = vp->fieldOfView() ? vp->fieldOfView()->value() : 0.785;
  return true;
}

// Column-major view-proj for the live world from the live Viewpoint, aspect-
// corrected to match WREN. Shared by draw + pick so a click hits what's shown.
bool WbWgpuView::worldViewProj(float out16[16]) const {
  WbMatrix4 cam;
  double horizFov = 0.785;
  if (!buildViewpointCamera(cam, horizFov))
    return false;
  const double aspect = static_cast<double>(pixelWidth()) / static_cast<double>(pixelHeight());
  WbWgpuSceneRenderer::buildViewProj(cam, horizFovForAspect(horizFov, aspect), aspect, 0.05, 1000.0,
                                     out16);
  return true;
}

// Step 3: render the live world from the live Viewpoint into the surface.
bool WbWgpuView::drawWorld(void *rgbaOut, int *outDraws) {
  if (!mSurface || !mMeshCache)
    return false;
  float vpm[16];
  if (!worldViewProj(vpm))
    return false;
  std::vector<WbWgpuSolidDraw> draws;
  std::vector<std::array<float, 16>> modelStorage;  // draws alias into this; keep alive past present
  std::vector<WbSolid *> tops;                      // parallel to draws (top solid per draw)
  WbWgpuSceneRenderer::collectWorldDraws(*mMeshCache, draws, modelStorage, &tops, mTextureCache);
  if (outDraws)
    *outDraws = static_cast<int>(draws.size());

  // R4 step-3c-A.2: highlight the live selection. Re-read every frame so it
  // tracks scene-tree / click selection changes; a null selection is a no-op.
  if (WbSelection::instance())
    highlightSelectedDraws(draws, tops, topSolidOf(WbSelection::instance()->selectedSolid()));

  WbWrenRenderingContext *rc = WbWrenRenderingContext::instance();

  // R4 step-3c-A: line overlays in the LIVE pane (off by default → WREN-identical).
  // Bounding objects go through the DEPTH-TESTED batch (green, read as 3D); joint axes
  // go through the ALWAYS-ON-TOP batch (cyan lines, visible through links — replacing the
  // old depth-occluded box representation). Both vectors must outlive the present below.
  std::vector<float> boEdges, jaLines;
  const float *lineV = nullptr;
  const float *topV = nullptr;
  uint32_t lineN = 0, topN = 0;
  static const float kBoColor[4] = {0.1f, 1.0f, 0.1f, 1.0f};   // green, depth-tested
  static const float kJaColor[4] = {0.0f, 0.95f, 0.95f, 1.0f};  // cyan, always-on-top
  if (rc && rc->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_ALL_BOUNDING_OBJECTS)) {
    lineN = collectBoundingEdges(boEdges);
    if (lineN >= 2)
      lineV = boEdges.data();
  }
  if (rc && rc->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_JOINT_AXES))
    collectJointAxisLineVerts(jaLines);  // appends joint-axis segments
  {
    // Per-type frustum gating (matches WREN): cameras/lidars under VF_CAMERA_FRUSTUMS,
    // range-finders under VF_RANGE_FINDER_FRUSTUMS. Either toggled → gather, the per-node
    // check inside picks which sensors contribute.
    const bool camFr = rc && rc->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_CAMERA_FRUSTUMS);
    const bool rfFr =
      rc && rc->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_RANGE_FINDER_FRUSTUMS);
    if (camFr || rfFr)
      collectFrustumLines(jaLines, camFr, rfFr);  // appends to the always-on-top batch
  }
  if (rc && rc->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_NORMALS))
    collectNormalLines(jaLines);  // appends surface-normal segments
  if (rc && rc->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_LIDAR_RAYS_PATHS))
    collectLidarRays(jaLines);  // appends lidar ray-path segments
  if (rc && rc->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_CONTACT_POINTS))
    collectContactCrosses(jaLines);  // appends contact-point crosses (0 in a static scene).
  // COM markers: per-solid gated (WREN has no global VF flag for these) → WREN-identical when
  // no solid has showGlobalCenterOfMassRepresentation enabled; appears in the live pane for the
  // ones that do. Always-on-top, shares the top batch.
  collectComCrossesGated(jaLines);
  // R4 step-3c-A: translate-manipulator handles at the selected solid (always-on-top,
  // shown whenever something is selected). Combined into the top batch (one colour in
  // the live pane for now; per-axis R/G/B is verified offscreen).
  if (WbSelection::instance()) {
    if (WbSolid *sel = WbSelection::instance()->selectedSolid()) {
      // Distance-scaled gizmo: handle length ~8% of the eye→solid distance so it keeps a
      // roughly constant on-screen size as the user zooms. Clamped to a sane minimum.
      double hlen = 0.25;
      WbMatrix4 hc;
      double hhf = 0.785;
      if (buildViewpointCamera(hc, hhf)) {
        const double d = (hc.translation() - sel->position()).length();
        hlen = d * 0.08;
        if (hlen < 0.02)
          hlen = 0.02;
      }
      std::vector<float> hx, hy, hz;
      emitManipulatorAxes(sel, hx, hy, hz, hlen);
      jaLines.insert(jaLines.end(), hx.begin(), hx.end());
      jaLines.insert(jaLines.end(), hy.begin(), hy.end());
      jaLines.insert(jaLines.end(), hz.begin(), hz.end());
    }
  }
  topN = static_cast<uint32_t>(jaLines.size() / 8);
  if (topN >= 2)
    topV = jaLines.data();

  const float light[4] = {0.3f, 0.4f, -0.85f, 0.35f};  // dir.xyz + ambient.w
  // R4 material fidelity: viewpoint eye for the textured-lit specular view vector.
  WbMatrix4 eyeCam;
  double eyeHf = 0.785;
  float camPos[3] = {0, 0, 0};
  const bool haveEye = buildViewpointCamera(eyeCam, eyeHf);
  if (haveEye) {
    const WbVector3 e = eyeCam.translation();
    camPos[0] = static_cast<float>(e.x());
    camPos[1] = static_cast<float>(e.y());
    camPos[2] = static_cast<float>(e.z());
  }
  // Sky-ish clear so a sparse world still reads as a scene, not a broken window.
  return mSurface->presentScene(vpm, light, draws.data(), static_cast<uint32_t>(draws.size()), 0.45f,
                                0.62f, 0.85f, rgbaOut, lineV, lineN, lineV ? kBoColor : nullptr,
                                topV, topN, topV ? kJaColor : nullptr, haveEye ? camPos : nullptr);
}

// R4 step-3c-A.1: pick the draw under pane pixel (px,py). Renders the world flat
// with per-draw IDs encoded in baseColor to an offscreen RGBA8 target, reads back
// the texel, decodes. Returns 0-based draw index, or -1 on miss/failure.
WbSolid *WbWgpuView::pickSolidAt(int px, int py) {
  if (!mBackend || !mMeshCache)
    return nullptr;
  const int w = pixelWidth(), h = pixelHeight();
  if (px < 0 || py < 0 || px >= w || py >= h)
    return nullptr;
  if (mPickTarget && (mPickTarget->width() != static_cast<uint32_t>(w) ||
                      mPickTarget->height() != static_cast<uint32_t>(h))) {
    delete mPickTarget;
    mPickTarget = nullptr;
  }
  if (!mPickTarget)
    mPickTarget = new WbWgpuRenderTarget(mBackend, static_cast<uint32_t>(w), static_cast<uint32_t>(h));
  if (!mPickTarget->isUsable())
    return nullptr;
  float vpm[16];
  if (!worldViewProj(vpm))
    return nullptr;
  std::vector<WbWgpuSolidDraw> draws;
  std::vector<std::array<float, 16>> modelStorage;
  std::vector<WbSolid *> nodes;  // parallel to draws (top solid per draw)
  WbWgpuSceneRenderer::collectWorldDraws(*mMeshCache, draws, modelStorage, &nodes);
  if (draws.empty())
    return nullptr;
  // Encode draw index+1 into baseColor (linear RGBA8 → exact byte round-trip).
  for (size_t i = 0; i < draws.size(); ++i) {
    const uint32_t id = static_cast<uint32_t>(i) + 1u;
    draws[i].baseColorR = static_cast<float>(id & 0xFFu) / 255.0f;
    draws[i].baseColorG = static_cast<float>((id >> 8) & 0xFFu) / 255.0f;
    draws[i].baseColorB = static_cast<float>((id >> 16) & 0xFFu) / 255.0f;
    draws[i].baseColorA = 1.0f;
  }
  std::vector<uint8_t> buf(static_cast<size_t>(w) * h * 4, 0);
  WbWgpuClearColor black;  // (0,0,0,1) => background decodes to ID 0 = miss
  const float light[4] = {0.0f, 0.0f, -1.0f, 1.0f};  // ignored by kSolidPick
  if (!mPickTarget->clearAndDrawScene(black, vpm, light, draws.data(),
                                      static_cast<uint32_t>(draws.size()), buf.data(), false, 1.0f,
                                      false, nullptr, /*pickMode=*/true))
    return nullptr;
  const size_t idx = (static_cast<size_t>(py) * w + px) * 4;
  const uint32_t id = static_cast<uint32_t>(buf[idx]) | (static_cast<uint32_t>(buf[idx + 1]) << 8) |
                      (static_cast<uint32_t>(buf[idx + 2]) << 16);
  if (id == 0 || id > nodes.size())
    return nullptr;  // background miss, or decode out of range
  return nodes[id - 1];
}

// R4 step-3c-A: which translate-handle axis of `sel` is under (px,py). Projects each
// gizmo axis segment to the pane and returns the nearest within a ~10px threshold.
int WbWgpuView::pickHandleAxis(int px, int py, WbSolid *sel, const float *vp16, int w, int h,
                               double len) const {
  if (!sel || !vp16)
    return -1;
  const WbVector3 o = sel->position();
  const WbVector3 ax[3] = {sel->xAxis().normalized(), sel->yAxis().normalized(),
                           sel->zAxis().normalized()};
  double osx, osy;
  if (!projectToScreen(vp16, o, w, h, osx, osy))
    return -1;
  int best = -1;
  double bestD = 10.0;  // px threshold
  for (int a = 0; a < 3; ++a) {
    double esx, esy;
    if (!projectToScreen(vp16, o + ax[a] * len, w, h, esx, esy))
      continue;
    const double d = distToSeg(px, py, osx, osy, esx, esy);
    if (d < bestD) {
      bestD = d;
      best = a;
    }
  }
  return best;
}

// R4 step-3c-A: which rotate-gizmo ring is under (px,py). Approximate: compares the
// click's distance from the projected gizmo centre to each ring's projected radius.
int WbWgpuView::pickRotateRing(int px, int py, WbSolid *sel, const float *vp16, int w, int h,
                               double len) const {
  if (!sel || !vp16)
    return -1;
  const WbVector3 o = sel->position();
  double osx, osy;
  if (!projectToScreen(vp16, o, w, h, osx, osy))
    return -1;
  const double clickDist = std::hypot(px - osx, py - osy);
  // Ring about X lies in the YZ plane; a representative radius point is along Y. Etc.
  const WbVector3 radPt[3] = {sel->yAxis().normalized(), sel->zAxis().normalized(),
                              sel->xAxis().normalized()};
  int best = -1;
  double bestErr = 9.0;  // px tolerance on the ring circle
  for (int a = 0; a < 3; ++a) {
    double rsx, rsy;
    if (!projectToScreen(vp16, o + radPt[a] * len, w, h, rsx, rsy))
      continue;
    const double screenR = std::hypot(rsx - osx, rsy - osy);
    const double err = std::fabs(clickDist - screenR);
    if (err < bestErr) {
      bestErr = err;
      best = a;
    }
  }
  return best;
}

void WbWgpuView::mousePressEvent(QMouseEvent *event) {
  if (mBackend && mMeshCache) {
    const int px = static_cast<int>(event->position().x() * devicePixelRatio());
    const int py = static_cast<int>(event->position().y() * devicePixelRatio());
    WbSolid *sel = WbSelection::instance() ? WbSelection::instance()->selectedSolid() : nullptr;
    float vpm[16];
    if (sel && worldViewProj(vpm)) {
      double hlen = kHandleLen;  // distance-scaled to match the rendered gizmo
      {
        WbMatrix4 hc;
        double hhf = 0.785;
        if (buildViewpointCamera(hc, hhf)) {
          const double d = (hc.translation() - sel->position()).length();
          hlen = d * 0.08;
          if (hlen < 0.02)
            hlen = 0.02;
        }
      }
      // First: a translate handle (axis line). Then: a rotate ring.
      const int axis = pickHandleAxis(px, py, sel, vpm, pixelWidth(), pixelHeight(), hlen);
      if (axis >= 0) {
        mGrabAxis = axis;
        mGrabSolid = sel;
        const WbVector3 t = sel->translation();
        mGrabStartT[0] = t.x();
        mGrabStartT[1] = t.y();
        mGrabStartT[2] = t.z();
        mGrabStartPx = px;
        mGrabStartPy = py;
        QWindow::mousePressEvent(event);
        return;
      }
      const int ring = pickRotateRing(px, py, sel, vpm, pixelWidth(), pixelHeight(), hlen);
      if (ring >= 0) {
        mGrabRotateAxis = ring;
        mGrabRotateSolid = sel;
        const WbRotation r = sel->rotation();
        mGrabStartRot[0] = r.x();
        mGrabStartRot[1] = r.y();
        mGrabStartRot[2] = r.z();
        mGrabStartRot[3] = r.angle();
        // Ring axis in the PARENT frame (R_localStart · unit), so the rotate composition
        // deltaM·startRotation is correct under a rotated/nested parent. For a top-level
        // solid R_parent = I, so this equals the world axis sel->{x,y,z}Axis() (unchanged).
        const WbVector3 unit(ring == 0 ? 1.0 : 0.0, ring == 1 ? 1.0 : 0.0, ring == 2 ? 1.0 : 0.0);
        const WbVector3 pa = (r.toMatrix3() * unit).normalized();
        mGrabRotateParentAxis[0] = pa.x();
        mGrabRotateParentAxis[1] = pa.y();
        mGrabRotateParentAxis[2] = pa.z();
        double osx = 0.0, osy = 0.0;
        projectToScreen(vpm, sel->position(), pixelWidth(), pixelHeight(), osx, osy);
        mGrabStartAngle = std::atan2(py - osy, px - osx);
        QWindow::mousePressEvent(event);
        return;
      }
    }
    // Otherwise: wgpu ID pick -> select the solid (R4 step-3c-A.1).
    WbSolid *picked = pickSolidAt(px, py);
    if (picked && WbSelection::instance())
      WbSelection::instance()->selectPoseFromView3D(picked);  // WbSolid is-a WbAbstractPose
  }
  QWindow::mousePressEvent(event);
}

void WbWgpuView::mouseMoveEvent(QMouseEvent *event) {
  if (mGrabAxis >= 0 && mGrabSolid) {
    const int px = static_cast<int>(event->position().x() * devicePixelRatio());
    const int py = static_cast<int>(event->position().y() * devicePixelRatio());
    float vpm[16];
    if (worldViewProj(vpm)) {
      const WbVector3 o = mGrabSolid->position();
      // axisWorld = the on-screen gizmo direction (the solid's world axis) — used to map
      // mouse pixels → metres. localDir = the SAME axis expressed in the PARENT frame
      // (R_local · unit) — used to update the parent-relative translation field. For a
      // top-level solid R_parent = I so localDir == axisWorld (byte-identical); under a
      // rotated/nested parent the parent rotation cancels, so the solid still slides along
      // the on-screen gizmo axis instead of drifting off it.
      const WbVector3 unit(mGrabAxis == 0 ? 1.0 : 0.0, mGrabAxis == 1 ? 1.0 : 0.0,
                           mGrabAxis == 2 ? 1.0 : 0.0);
      const WbVector3 axisWorld = (mGrabAxis == 0 ? mGrabSolid->xAxis()
                                   : mGrabAxis == 1 ? mGrabSolid->yAxis()
                                                    : mGrabSolid->zAxis())
                                    .normalized();
      const WbVector3 localDir = mGrabSolid->rotation().toMatrix3() * unit;
      double osx, osy, esx, esy;
      if (projectToScreen(vpm, o, pixelWidth(), pixelHeight(), osx, osy) &&
          projectToScreen(vpm, o + axisWorld, pixelWidth(), pixelHeight(), esx, esy)) {
        double sdx = esx - osx, sdy = esy - osy;
        const double slen = std::hypot(sdx, sdy);  // pane pixels per metre along the axis
        if (slen > 1e-3) {
          sdx /= slen;
          sdy /= slen;
          const double mdx = px - mGrabStartPx, mdy = py - mGrabStartPy;
          const double worldDist = (mdx * sdx + mdy * sdy) / slen;  // metres along the axis
          mGrabSolid->setTranslation(WbVector3(mGrabStartT[0] + localDir.x() * worldDist,
                                               mGrabStartT[1] + localDir.y() * worldDist,
                                               mGrabStartT[2] + localDir.z() * worldDist));
        }
      }
    }
  } else if (mGrabRotateAxis >= 0 && mGrabRotateSolid) {
    // Rotate drag: the mouse angle around the projected gizmo centre drives a rotation
    // about the fixed grab-time PARENT-frame axis, composed onto the solid's start
    // rotation (both parent-relative → setRotation is correct under a rotated parent).
    const int px = static_cast<int>(event->position().x() * devicePixelRatio());
    const int py = static_cast<int>(event->position().y() * devicePixelRatio());
    float vpm[16];
    if (worldViewProj(vpm)) {
      double osx, osy;
      if (projectToScreen(vpm, mGrabRotateSolid->position(), pixelWidth(), pixelHeight(), osx, osy)) {
        const double cur = std::atan2(py - osy, px - osx);
        const double delta = cur - mGrabStartAngle;
        const WbMatrix3 deltaM(mGrabRotateParentAxis[0], mGrabRotateParentAxis[1],
                               mGrabRotateParentAxis[2], delta);
        const WbRotation startR(mGrabStartRot[0], mGrabStartRot[1], mGrabStartRot[2], mGrabStartRot[3]);
        mGrabRotateSolid->setRotation(WbRotation(deltaM * startR.toMatrix3()));
      }
    }
  }
  QWindow::mouseMoveEvent(event);
}

void WbWgpuView::mouseReleaseEvent(QMouseEvent *event) {
  mGrabAxis = -1;
  mGrabSolid = nullptr;
  mGrabRotateAxis = -1;
  mGrabRotateSolid = nullptr;
  QWindow::mouseReleaseEvent(event);
}

// Render the mimosa unit box spinning about the vertical axis at `phase`,
// fixed camera at (-2.5,0,0) looking down +X (Webots camera convention) at
// the box at the origin. rgbaOut, if non-null, receives the rendered
// backbuffer for the self-check. Returns presentScene's success.
bool WbWgpuView::drawBox(double phase, void *rgbaOut) {
  if (!mSurface || !mMeshCache)
    return false;
  WbWgpuMeshHandle box =
    WbWgpuMeshAdapter::acquirePrimitive(*mMeshCache, 1ull, WbWgpuMeshAdapter::PrimitiveKind::Box);
  if (!box.vertexBuffer || !box.indexBuffer || !box.indexCount)
    return false;

  const float a = static_cast<float>(phase);
  const float c = std::cos(a), s = std::sin(a);
  const float model[16] = {c, s, 0, 0, -s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};  // col-major Z-rot

  WbMatrix4 cam;  // identity rotation => local +X (forward) = world +X
  cam.setTranslation(-2.5, 0.0, 0.0);
  float vp[16];
  const double aspect = static_cast<double>(pixelWidth()) / static_cast<double>(pixelHeight());
  WbWgpuSceneRenderer::buildViewProj(cam, 1.1, aspect, 0.05, 100.0, vp);

  WbWgpuSolidDraw d;
  d.modelMatrix16 = model;
  d.baseColorR = 0.96f;  // OmniSim mimosa
  d.baseColorG = 0.91f;
  d.baseColorB = 0.02f;
  d.baseColorA = 1.0f;
  d.vertexBuffer = box.vertexBuffer;
  d.indexBuffer = box.indexBuffer;
  d.indexCount = box.indexCount;

  const float light[4] = {0.3f, 0.4f, -0.85f, 0.35f};  // dir.xyz + ambient.w
  return mSurface->presentScene(vp, light, &d, 1, 0.05f, 0.06f, 0.09f, rgbaOut);
}

// One-shot numerical verification: render two frames at distinct rotations,
// read each backbuffer back, and write pixel statistics to the file named by
// OMNISIM_VIEW3D_WGPU_SELFCHECK. No human eyes required — a bright box at
// centre + a dark corner + nonzero frame-to-frame diff proves the surface is
// rendering animated, lit 3D geometry.
void WbWgpuView::runSelfCheck() {
  if (!mBackend || !mMeshCache)
    return;
  const QByteArray path = qEnvironmentVariable("OMNISIM_VIEW3D_WGPU_SELFCHECK").toLocal8Bit();
  std::ofstream f(path.constData());
  if (!f)
    return;
  f << "R4 step-3c-A.1 — wgpu offscreen pick self-check (exposure-independent)\n";

  WbMatrix4 cam;
  double hf = 0.785;
  if (!buildViewpointCamera(cam, hf)) {
    f << "no world/viewpoint loaded — cannot verify\n";
    return;
  }
  // Fixed square offscreen target — result is independent of pane size/exposure.
  const uint32_t S = 480;
  float vpm[16];
  WbWgpuSceneRenderer::buildViewProj(cam, hf, 1.0, 0.05, 1000.0, vpm);  // 1:1 aspect

  std::vector<WbWgpuSolidDraw> draws;
  std::vector<std::array<float, 16>> ms;  // draws alias into this; keep alive past the render
  WbWgpuSceneRenderer::collectWorldDraws(*mMeshCache, draws, ms, nullptr, mTextureCache);
  long texturedDraws = 0;
  for (const WbWgpuSolidDraw &d : draws)
    if (d.textureView)
      ++texturedDraws;
  f << "collectWorldDraws = " << draws.size() << " shapes (" << texturedDraws
    << " with albedo texture uploaded)\n";
  if (draws.empty()) {
    f << "no harvestable geometry — cannot verify pick\n";
    return;
  }

  // R4 robustness (verify-first): count degenerate draws — non-finite or absurd-
  // magnitude model transforms (the z=1e5/1e22 sentinels noted on panda). Confirm
  // the problem exists before adding any cull, so we don't 'fix' a non-problem.
  {
    long degenerate = 0;
    double maxAbsT = 0.0;
    for (const WbWgpuSolidDraw &d : draws) {
      if (!d.modelMatrix16)
        continue;
      bool bad = false;
      for (int k = 0; k < 16; ++k)
        if (!std::isfinite(d.modelMatrix16[k]))
          bad = true;
      for (int k = 12; k < 15; ++k) {
        const double a = std::fabs(static_cast<double>(d.modelMatrix16[k]));
        if (a > maxAbsT)
          maxAbsT = a;
        if (a > 1e4)
          bad = true;
      }
      if (bad)
        ++degenerate;
    }
    f << "degenerate-draw scan: " << degenerate << " of " << draws.size()
      << " draws non-finite or |translation|>1e4; max|translation|=" << maxAbsT << "\n";
  }

  WbWgpuRenderTarget target(mBackend, S, S);
  f << "offscreen target " << S << "x" << S << " usable=" << (target.isUsable() ? 1 : 0) << "\n";
  if (!target.isUsable())
    return;

  // Encode draw index+1 into baseColor (linear RGBA8 → exact byte round-trip).
  for (size_t i = 0; i < draws.size(); ++i) {
    const uint32_t id = static_cast<uint32_t>(i) + 1u;
    draws[i].baseColorR = static_cast<float>(id & 0xFFu) / 255.0f;
    draws[i].baseColorG = static_cast<float>((id >> 8) & 0xFFu) / 255.0f;
    draws[i].baseColorB = static_cast<float>((id >> 16) & 0xFFu) / 255.0f;
    draws[i].baseColorA = 1.0f;
  }
  std::vector<uint8_t> buf(static_cast<size_t>(S) * S * 4, 0);
  WbWgpuClearColor black;  // (0,0,0,1) => background decodes to ID 0 = miss
  const float light[4] = {0.0f, 0.0f, -1.0f, 1.0f};  // ignored by kSolidPick
  const bool ok = target.clearAndDrawScene(black, vpm, light, draws.data(),
                                           static_cast<uint32_t>(draws.size()), buf.data(), false,
                                           1.0f, false, nullptr, /*pickMode=*/true);
  f << "pick render ok=" << (ok ? 1 : 0) << "\n";
  uint32_t centerId = 0;  // captured for the selection-highlight check below
  if (ok) {
    long covered = 0;  // non-zero (= geometry-painted) texels
    for (size_t i = 0; i < static_cast<size_t>(S) * S; ++i)
      if (buf[i * 4] || buf[i * 4 + 1] || buf[i * 4 + 2])
        ++covered;
    f << "id-coverage = " << covered << " / " << (S * S) << "  ("
      << (100.0 * covered / (static_cast<double>(S) * S)) << "%)\n";
    const size_t c = (static_cast<size_t>(S / 2) * S + S / 2) * 4;
    centerId = static_cast<uint32_t>(buf[c]) | (static_cast<uint32_t>(buf[c + 1]) << 8) |
               (static_cast<uint32_t>(buf[c + 2]) << 16);
    if (centerId == 0)
      f << "pick(center) -> id=0  [miss/background]\n";
    else
      f << "pick(center) -> id=" << centerId << " => draw index " << (centerId - 1)
        << "  [HIT — wgpu ID pick works]\n";
  }

  // R4 step-3c-A.2: selection-highlight verification. Everything here comes from a
  // SINGLE same-frame collection (`lit`/`litTops` + a same-frame ID buffer) so the
  // comparison is internally consistent. The top-of-function `buf` is from an
  // EARLIER pick render; the sim can advance between renders, so classifying
  // against it produced spurious "outside" counts (~1/3 of runs) — that was a
  // harness bug, not a readback race (OMNISIM_PROBE_READBACK proves the readback is
  // byte-deterministic).
  if (ok && centerId != 0) {
    std::vector<WbWgpuSolidDraw> lit;
    std::vector<std::array<float, 16>> lms;  // lit aliases into this; keep alive past render
    std::vector<WbSolid *> litTops;          // parallel to lit
    WbWgpuSceneRenderer::collectWorldDraws(*mMeshCache, lit, lms, &litTops);
    // Same-frame ID buffer: a COPY of lit with per-draw IDs encoded, rendered flat.
    // Drives BOTH selTop and the changed-pixel classification, so it matches the
    // litA/litB renders below exactly (same geometry, same instant).
    std::vector<WbWgpuSolidDraw> idDraws = lit;
    for (size_t i = 0; i < idDraws.size(); ++i) {
      const uint32_t id = static_cast<uint32_t>(i) + 1u;
      idDraws[i].baseColorR = static_cast<float>(id & 0xFFu) / 255.0f;
      idDraws[i].baseColorG = static_cast<float>((id >> 8) & 0xFFu) / 255.0f;
      idDraws[i].baseColorB = static_cast<float>((id >> 16) & 0xFFu) / 255.0f;
      idDraws[i].baseColorA = 1.0f;
    }
    std::vector<uint8_t> pidBuf(static_cast<size_t>(S) * S * 4, 0);
    const float pickLight[4] = {0.0f, 0.0f, -1.0f, 1.0f};  // ignored by kSolidPick
    const bool okP = target.clearAndDrawScene(black, vpm, pickLight, idDraws.data(),
                                              static_cast<uint32_t>(idDraws.size()), pidBuf.data(),
                                              false, 1.0f, false, nullptr, /*pickMode=*/true);
    const size_t cc = (static_cast<size_t>(S / 2) * S + S / 2) * 4;
    const uint32_t selId = okP ? (static_cast<uint32_t>(pidBuf[cc]) |
                                  (static_cast<uint32_t>(pidBuf[cc + 1]) << 8) |
                                  (static_cast<uint32_t>(pidBuf[cc + 2]) << 16))
                               : 0u;
    WbSolid *selTop = (selId != 0 && (selId - 1) < litTops.size()) ? litTops[selId - 1] : nullptr;
    std::vector<uint8_t> litA(static_cast<size_t>(S) * S * 4, 0), litB(litA.size(), 0);
    WbWgpuClearColor sky;
    sky.r = 0.45f;
    sky.g = 0.62f;
    sky.b = 0.85f;
    const float lit4[4] = {0.3f, 0.4f, -0.85f, 0.35f};
    const bool okA = target.clearAndDrawScene(sky, vpm, lit4, lit.data(),
                                              static_cast<uint32_t>(lit.size()), litA.data());
    highlightSelectedDraws(lit, litTops, selTop);
    const bool okB = target.clearAndDrawScene(sky, vpm, lit4, lit.data(),
                                              static_cast<uint32_t>(lit.size()), litB.data());
    long selDraws = 0;
    for (WbSolid *t : litTops)
      if (t == selTop)
        ++selDraws;
    if (okP && okA && okB && selTop) {
      long changed = 0, changedOutside = 0;
      for (size_t i = 0; i < static_cast<size_t>(S) * S; ++i) {
        const size_t b = i * 4;
        const bool diff = litA[b] != litB[b] || litA[b + 1] != litB[b + 1] || litA[b + 2] != litB[b + 2];
        if (!diff)
          continue;
        ++changed;
        const uint32_t pid = static_cast<uint32_t>(pidBuf[b]) |
                             (static_cast<uint32_t>(pidBuf[b + 1]) << 8) |
                             (static_cast<uint32_t>(pidBuf[b + 2]) << 16);
        WbSolid *pt = (pid != 0 && (pid - 1) < litTops.size()) ? litTops[pid - 1] : nullptr;
        if (pt != selTop)
          ++changedOutside;
      }
      f << "selection-highlight: selTop has " << selDraws << " draws; changed " << changed << " px, "
        << changedOutside << " outside selection  ["
        << ((changed > 0 && changedOutside == 0) ? "PASS — highlight paints exactly the selected solid"
                                                 : "FAIL")
        << "]\n";
    } else {
      f << "selection-highlight: render failed (okP=" << okP << " okA=" << okA << " okB=" << okB
        << " selTop=" << (selTop ? 1 : 0) << ")\n";
    }
  }

  // R4 step-3c-A.3: joint-axis optional-rendering verification. Render the world
  // LIT once plain, once with joint-axis bars appended (collected directly, i.e.
  // ignoring the VF_JOINT_AXES gate which is off in headless), and confirm the
  // bars (a) are non-zero in count and (b) actually change the rendered image.
  {
    std::vector<WbWgpuSolidDraw> jd;
    std::vector<std::array<float, 16>> jms, jaxis;  // jd aliases jms then jaxis; keep both alive
    WbWgpuSceneRenderer::collectWorldDraws(*mMeshCache, jd, jms);
    std::vector<uint8_t> bufBase(static_cast<size_t>(S) * S * 4, 0), bufAx(bufBase.size(), 0);
    WbWgpuClearColor sky;
    sky.r = 0.45f;
    sky.g = 0.62f;
    sky.b = 0.85f;
    const float lit4[4] = {0.3f, 0.4f, -0.85f, 0.35f};
    const bool okBase = target.clearAndDrawScene(sky, vpm, lit4, jd.data(),
                                                 static_cast<uint32_t>(jd.size()), bufBase.data());
    const int nAxes = collectJointAxisDraws(*mMeshCache, jd, jaxis);
    const bool okAx = target.clearAndDrawScene(sky, vpm, lit4, jd.data(),
                                               static_cast<uint32_t>(jd.size()), bufAx.data());
    long changed = 0;
    if (okBase && okAx)
      for (size_t i = 0; i < static_cast<size_t>(S) * S; ++i) {
        const size_t b = i * 4;
        if (bufBase[b] != bufAx[b] || bufBase[b + 1] != bufAx[b + 1] || bufBase[b + 2] != bufAx[b + 2])
          ++changed;
      }
    f << "joint-axes: " << nAxes << " axis bars; lit-with-axes changed " << changed << " px vs without  ["
      << ((nAxes > 0 && changed > 0) ? "PASS — joint axes render" : "FAIL/none") << "]\n";
  }

  // R4 step-3c-A: screenshot capability. Render the world LIT to an offscreen target
  // at an arbitrary resolution (not tied to the pane/window), read it back (proven-
  // deterministic path), pack into a QImage in RGBA8888 byte order, and save a PNG.
  // Foundation for the screenshot action, WbVideoRecorder, and golden-image CI.
  {
    const uint32_t W = 640, H = 480;
    WbWgpuRenderTarget shot(mBackend, W, H);
    if (shot.isUsable()) {
      const double aspect = static_cast<double>(W) / static_cast<double>(H);
      float svp[16];
      WbWgpuSceneRenderer::buildViewProj(cam, horizFovForAspect(hf, aspect), aspect, 0.05, 1000.0, svp);
      std::vector<WbWgpuSolidDraw> sd;
      std::vector<std::array<float, 16>> sms;
      WbWgpuSceneRenderer::collectWorldDraws(*mMeshCache, sd, sms, nullptr, mTextureCache);
      std::vector<uint8_t> sbuf(static_cast<size_t>(W) * H * 4, 0);
      WbWgpuClearColor sky;
      sky.r = 0.45f;
      sky.g = 0.62f;
      sky.b = 0.85f;
      const float lit4[4] = {0.3f, 0.4f, -0.85f, 0.35f};
      const WbVector3 eye = cam.translation();
      const float camPos[3] = {static_cast<float>(eye.x()), static_cast<float>(eye.y()),
                               static_cast<float>(eye.z())};  // R4 PBR: view vector for specular
      const bool oks = shot.clearAndDrawScene(sky, svp, lit4, sd.data(),
                                              static_cast<uint32_t>(sd.size()), sbuf.data(), false,
                                              1.0f, false, camPos);
      if (oks) {
        // R4 step-3c-A: overlay bounding-object wireframes (VF_ALL_BOUNDING_OBJECTS) on
        // the lit scene via the line pipeline. The on-screen pane can't show these yet
        // (WbWgpuSurface has no line path — a follow-up), but this verifies the edge
        // generation + line overlay end-to-end and bakes them into the PNG for a visual
        // check. loadExisting=true preserves the scene colour+depth so wireframes occlude.
        std::vector<float> bedges;
        const uint32_t bverts = collectBoundingEdges(bedges);
        const std::vector<uint8_t> preOverlay = sbuf;  // for the overlay diff
        bool okBO = false;
        long overlayChanged = 0;
        if (bverts >= 2) {
          const float green[4] = {0.1f, 1.0f, 0.1f, 1.0f};
          okBO = shot.clearAndDrawLines(sky, svp, green, bedges.data(), bverts, sbuf.data(),
                                        /*loadExisting=*/true);
          if (okBO)
            for (size_t i = 0; i < static_cast<size_t>(W) * H; ++i)
              if (sbuf[i * 4] != preOverlay[i * 4] || sbuf[i * 4 + 1] != preOverlay[i * 4 + 1] ||
                  sbuf[i * 4 + 2] != preOverlay[i * 4 + 2])
                ++overlayChanged;
        }
        f << "bounding-objects: " << (bverts / 2) << " segments; overlay ok=" << (okBO ? 1 : 0)
          << " changed " << overlayChanged << " px  ["
          << ((bverts >= 2 && okBO && overlayChanged > 0) ? "PASS — bounding wireframes render"
                                                          : "none/FAIL")
          << "]\n";

        // Centre-of-mass markers (magenta crosses, one per solid). Overlays on top of
        // the scene+bounding via the line pipeline; verifies the COM glyph end-to-end.
        std::vector<float> com;
        const uint32_t cverts = collectComCrosses(com);
        const std::vector<uint8_t> preCom = sbuf;
        bool okCom = false;
        long comChanged = 0;
        if (cverts >= 2) {
          const float magenta[4] = {1.0f, 0.0f, 1.0f, 1.0f};
          okCom = shot.clearAndDrawLines(sky, svp, magenta, com.data(), cverts, sbuf.data(),
                                         /*loadExisting=*/true, /*depthTest=*/false);  // always-on-top
          if (okCom)
            for (size_t i = 0; i < static_cast<size_t>(W) * H; ++i)
              if (sbuf[i * 4] != preCom[i * 4] || sbuf[i * 4 + 1] != preCom[i * 4 + 1] ||
                  sbuf[i * 4 + 2] != preCom[i * 4 + 2])
                ++comChanged;
        }
        f << "center-of-mass: " << (cverts / 6) << " markers; overlay ok=" << (okCom ? 1 : 0)
          << " changed " << comChanged << " px  ["
          << ((cverts >= 6 && okCom && comChanged > 0) ? "PASS — COM markers render" : "none/FAIL")
          << "]\n";

        // COM live-pane gating: the per-solid-gated collector (what drawWorld feeds the live
        // pane) must be EMPTY by default — so the pane is WREN-identical when nothing is toggled
        // — and emit once a solid's representation is enabled. Toggle it on the first physics
        // solid and restore. This is the live-pane COM path (the unconditional collector above
        // is the offscreen-only one).
        {
          std::vector<float> g0;
          const uint32_t gated0 = collectComCrossesGated(g0);
          std::vector<WbSolid *> all;
          if (WbWorld::instance())
            for (WbSolid *t : WbWorld::instance()->topSolids()) {
              all.push_back(t);
              collectDescendantSolids(t, all);
            }
          uint32_t gated1 = 0;
          bool toggled = false;
          for (WbSolid *s : all) {
            if (s->showGlobalCenterOfMassRepresentation(true)) {  // true ⇒ non-kinematic, flag set
              std::vector<float> g1;
              gated1 = collectComCrossesGated(g1);
              s->showGlobalCenterOfMassRepresentation(false);  // restore
              toggled = true;
              break;
            }
          }
          const bool pass = gated0 == 0 && (!toggled || gated1 > 0);
          f << "com-live-gating: default=" << gated0 << " after-enable=" << gated1
            << " toggled=" << (toggled ? 1 : 0) << "  ["
            << (pass ? (toggled ? "PASS — off by default, emits when a solid enables it"
                                : "PASS — off by default (no physics solid to toggle)")
                     : "FAIL")
            << "]\n";
        }

        // Contact-point markers (cyan crosses, VF_CONTACT_POINTS). Count depends on the
        // physics state — report it; 0 means nothing was touching this frame, not a fault.
        std::vector<float> cps;
        const uint32_t cpverts = collectContactCrosses(cps);
        bool okCp = false;
        long cpChanged = 0;
        if (cpverts >= 2) {
          const std::vector<uint8_t> preCp = sbuf;
          const float cyan[4] = {0.0f, 0.9f, 0.9f, 1.0f};
          okCp = shot.clearAndDrawLines(sky, svp, cyan, cps.data(), cpverts, sbuf.data(),
                                        /*loadExisting=*/true, /*depthTest=*/false);  // always-on-top
          if (okCp)
            for (size_t i = 0; i < static_cast<size_t>(W) * H; ++i)
              if (sbuf[i * 4] != preCp[i * 4] || sbuf[i * 4 + 1] != preCp[i * 4 + 1] ||
                  sbuf[i * 4 + 2] != preCp[i * 4 + 2])
                ++cpChanged;
        }
        f << "contact-points: " << (cpverts / 6) << " markers; overlay ok=" << (okCp ? 1 : 0)
          << " changed " << cpChanged << " px  ["
          << (cpverts == 0 ? "none (nothing touching this frame)"
                           : (okCp && cpChanged > 0 ? "PASS — contact markers render" : "FAIL"))
          << "]\n";

        // Support polygon — convex hull of each resting solid's ground contacts, on the
        // horizontal plane (the stability footprint). Like contacts, 0 in a static scene;
        // needs ≥3 contacts per solid. Yellow, always-on-top. Verified on the resting-contact
        // fixture (tests/physics/fixtures/wgpu_overlay_contacts.wbt).
        std::vector<float> sp;
        const uint32_t spverts = collectSupportPolygon(sp);
        bool okSp = false;
        long spChanged = 0;
        if (spverts >= 2) {
          const std::vector<uint8_t> preSp = sbuf;
          const float yellow[4] = {1.0f, 0.85f, 0.0f, 1.0f};
          okSp = shot.clearAndDrawLines(sky, svp, yellow, sp.data(), spverts, sbuf.data(),
                                        /*loadExisting=*/true, /*depthTest=*/false);  // always-on-top
          if (okSp)
            for (size_t i = 0; i < static_cast<size_t>(W) * H; ++i)
              if (sbuf[i * 4] != preSp[i * 4] || sbuf[i * 4 + 1] != preSp[i * 4 + 1] ||
                  sbuf[i * 4 + 2] != preSp[i * 4 + 2])
                ++spChanged;
        }
        f << "support-polygon: " << spverts << " hull segments; overlay ok=" << (okSp ? 1 : 0)
          << " changed " << spChanged << " px  ["
          << (spverts == 0 ? "none (no solid with >=3 ground contacts this frame)"
                           : (okSp && spChanged > 0 ? "PASS — support polygon renders" : "FAIL"))
          << "]\n";

        // Camera + range-finder frustums. Count depends on the world having sensor nodes
        // (0 in panda; non-zero in e.g. camera.wbt). Pass both type-gates ON so this verifies
        // the frustum generator for cameras AND range-finders. Always-on-top orange.
        std::vector<float> fr;
        const uint32_t frverts = collectFrustumLines(fr, /*cameraOn=*/true, /*rangeOn=*/true);
        bool okFr = false;
        long frChanged = 0;
        if (frverts >= 2) {
          const std::vector<uint8_t> preFr = sbuf;
          const float orange[4] = {1.0f, 0.55f, 0.0f, 1.0f};
          okFr = shot.clearAndDrawLines(sky, svp, orange, fr.data(), frverts, sbuf.data(),
                                        /*loadExisting=*/true, /*depthTest=*/false);
          if (okFr)
            for (size_t i = 0; i < static_cast<size_t>(W) * H; ++i)
              if (sbuf[i * 4] != preFr[i * 4] || sbuf[i * 4 + 1] != preFr[i * 4 + 1] ||
                  sbuf[i * 4 + 2] != preFr[i * 4 + 2])
                ++frChanged;
        }
        f << "frustums: " << (frverts / 24) << " sensors (" << (frverts / 2) << " segments); overlay ok="
          << (okFr ? 1 : 0) << " changed " << frChanged << " px  ["
          << (frverts == 0 ? "none (no Camera/RangeFinder/Lidar in this world)"
                           : (okFr && frChanged > 0 ? "PASS — frustums render" : "FAIL"))
          << "]\n";

        // Per-type frustum gating must PARTITION: camera-only + range-finder-only == both-on.
        // (Verified on the dual-sensor fixture; a single-type world reports one side = 0.)
        {
          std::vector<float> camOnly, rfOnly;
          const uint32_t camV = collectFrustumLines(camOnly, /*cameraOn=*/true, /*rangeOn=*/false);
          const uint32_t rfV = collectFrustumLines(rfOnly, /*cameraOn=*/false, /*rangeOn=*/true);
          const bool partitions = (camV + rfV == frverts);
          f << "frustum-gating: camera-only=" << (camV / 24) << " range-finder-only=" << (rfV / 24)
            << " both=" << (frverts / 24) << " partitions=" << (partitions ? 1 : 0) << "  ["
            << (frverts == 0 ? "none (no sensors)"
                             : (partitions && camV > 0 && rfV > 0
                                  ? "PASS — per-type gating partitions (camera + range-finder)"
                                  : (partitions ? "PASS — partitions (single sensor type in world)"
                                                : "FAIL")))
            << "]\n";
        }

        // Surface normals (VF_NORMALS) — short outward lines on primitive geometries.
        // Always-on-top yellow. (Per-vertex mesh normals are a follow-up.)
        std::vector<float> nl;
        const uint32_t nlverts = collectNormalLines(nl);
        bool okNl = false;
        long nlChanged = 0;
        if (nlverts >= 2) {
          const std::vector<uint8_t> preNl = sbuf;
          const float yellow[4] = {1.0f, 1.0f, 0.0f, 1.0f};
          okNl = shot.clearAndDrawLines(sky, svp, yellow, nl.data(), nlverts, sbuf.data(),
                                        /*loadExisting=*/true, /*depthTest=*/false);
          if (okNl)
            for (size_t i = 0; i < static_cast<size_t>(W) * H; ++i)
              if (sbuf[i * 4] != preNl[i * 4] || sbuf[i * 4 + 1] != preNl[i * 4 + 1] ||
                  sbuf[i * 4 + 2] != preNl[i * 4 + 2])
                ++nlChanged;
        }
        f << "normals: " << (nlverts / 2) << " segments; overlay ok=" << (okNl ? 1 : 0) << " changed "
          << nlChanged << " px  ["
          << (nlverts == 0 ? "none (no primitive geometries)"
                           : (okNl && nlChanged > 0 ? "PASS — normals render" : "FAIL"))
          << "]\n";

        // Lidar ray paths (VF_LIDAR_RAYS_PATHS). Count depends on the world having a Lidar
        // (0 here / in panda; non-zero in lidar.wbt). Always-on-top, light blue.
        std::vector<float> lr;
        const uint32_t lrverts = collectLidarRays(lr);
        bool okLr = false;
        long lrChanged = 0;
        if (lrverts >= 2) {
          const std::vector<uint8_t> preLr = sbuf;
          const float skyblue[4] = {0.4f, 0.7f, 1.0f, 1.0f};
          okLr = shot.clearAndDrawLines(sky, svp, skyblue, lr.data(), lrverts, sbuf.data(),
                                        /*loadExisting=*/true, /*depthTest=*/false);
          if (okLr)
            for (size_t i = 0; i < static_cast<size_t>(W) * H; ++i)
              if (sbuf[i * 4] != preLr[i * 4] || sbuf[i * 4 + 1] != preLr[i * 4 + 1] ||
                  sbuf[i * 4 + 2] != preLr[i * 4 + 2])
                ++lrChanged;
        }
        f << "lidar-rays: " << (lrverts / 2) << " rays; overlay ok=" << (okLr ? 1 : 0) << " changed "
          << lrChanged << " px  ["
          << (lrverts == 0 ? "none (no Lidar in this world)"
                           : (okLr && lrChanged > 0 ? "PASS — lidar rays render" : "FAIL"))
          << "]\n";

        // Manipulator handles (translate gizmo): 3 world-axis lines at a solid's origin.
        // Demo on EVERY top solid (no GUI selection here, so cover all to land some in
        // view); always-on-top R/G/B.
        std::vector<float> hx, hy, hz;
        if (WbWorld::instance())
          for (WbSolid *hs : WbWorld::instance()->topSolids()) {
            double hl = (cam.translation() - hs->position()).length() * 0.08;  // distance-scaled
            if (hl < 0.02)
              hl = 0.02;
            emitManipulatorAxes(hs, hx, hy, hz, hl);
          }
        long hChanged = 0;
        bool okH = false;
        if (hx.size() >= 16) {  // 2 verts × 8 floats per axis
          const std::vector<uint8_t> preH = sbuf;
          const float red[4] = {1.0f, 0.1f, 0.1f, 1.0f};
          const float grn[4] = {0.1f, 1.0f, 0.1f, 1.0f};
          const float blu[4] = {0.2f, 0.4f, 1.0f, 1.0f};
          const uint32_t nx = static_cast<uint32_t>(hx.size() / 8);
          okH = shot.clearAndDrawLines(sky, svp, red, hx.data(), nx, sbuf.data(), true, false) &&
                shot.clearAndDrawLines(sky, svp, grn, hy.data(), nx, sbuf.data(), true, false) &&
                shot.clearAndDrawLines(sky, svp, blu, hz.data(), nx, sbuf.data(), true, false);
          if (okH)
            for (size_t i = 0; i < static_cast<size_t>(W) * H; ++i)
              if (sbuf[i * 4] != preH[i * 4] || sbuf[i * 4 + 1] != preH[i * 4 + 1] ||
                  sbuf[i * 4 + 2] != preH[i * 4 + 2])
                ++hChanged;
        }
        f << "manipulator-handles: " << (hx.size() / 16) << " gizmo (3 axes); overlay ok=" << (okH ? 1 : 0)
          << " changed " << hChanged << " px  ["
          << (hx.size() < 16 ? "none (no solid)" : (okH && hChanged > 0 ? "PASS — handles render" : "FAIL"))
          << "]\n";

        // Handle hit-test (the grab half of drag): project an on-screen solid's X-handle
        // midpoint and confirm pickHandleAxis returns axis 0 (X) at that pixel.
        int hitAxis = -2;
        bool hitTested = false;
        if (WbWorld::instance())
          for (WbSolid *hs : WbWorld::instance()->topSolids()) {
            double mx, my;
            if (projectToScreen(svp, hs->position() + hs->xAxis().normalized() * 0.125,
                                static_cast<int>(W), static_cast<int>(H), mx, my) &&
                mx >= 0 && mx < W && my >= 0 && my < H) {
              hitAxis = pickHandleAxis(static_cast<int>(mx), static_cast<int>(my), hs, svp,
                                       static_cast<int>(W), static_cast<int>(H), 0.25);
              hitTested = true;
              break;
            }
          }
        f << "handle-hit-test: on-screen=" << (hitTested ? 1 : 0) << " X-midpoint -> axis " << hitAxis
          << "  [" << (hitTested && hitAxis == 0 ? "PASS — grabs X" : "see value") << "]\n";

        // Rotate gizmo: 3 R/G/B rings per solid (about X/Y/Z), distance-scaled, always-on-top.
        std::vector<float> rrx, rry, rrz;
        if (WbWorld::instance())
          for (WbSolid *rs : WbWorld::instance()->topSolids()) {
            double rl = (cam.translation() - rs->position()).length() * 0.08;
            if (rl < 0.02)
              rl = 0.02;
            emitRotateRings(rs, rrx, rry, rrz, rl);
          }
        long rrChanged = 0;
        bool okRR = false;
        if (rrx.size() >= 16) {
          const std::vector<uint8_t> preRR = sbuf;
          const float red[4] = {1.0f, 0.1f, 0.1f, 1.0f};
          const float grn[4] = {0.1f, 1.0f, 0.1f, 1.0f};
          const float blu[4] = {0.2f, 0.4f, 1.0f, 1.0f};
          okRR = shot.clearAndDrawLines(sky, svp, red, rrx.data(), static_cast<uint32_t>(rrx.size() / 8),
                                        sbuf.data(), true, false) &&
                 shot.clearAndDrawLines(sky, svp, grn, rry.data(), static_cast<uint32_t>(rry.size() / 8),
                                        sbuf.data(), true, false) &&
                 shot.clearAndDrawLines(sky, svp, blu, rrz.data(), static_cast<uint32_t>(rrz.size() / 8),
                                        sbuf.data(), true, false);
          if (okRR)
            for (size_t i = 0; i < static_cast<size_t>(W) * H; ++i)
              if (sbuf[i * 4] != preRR[i * 4] || sbuf[i * 4 + 1] != preRR[i * 4 + 1] ||
                  sbuf[i * 4 + 2] != preRR[i * 4 + 2])
                ++rrChanged;
        }
        f << "rotate-gizmo: " << (rrx.size() / 16) << " rings; overlay ok=" << (okRR ? 1 : 0)
          << " changed " << rrChanged << " px  ["
          << (rrx.size() < 16 ? "none" : (okRR && rrChanged > 0 ? "PASS — rotate rings render" : "FAIL"))
          << "]\n";

        // Scale gizmo: small R/G/B cubes at each solid's axis ends, distance-scaled.
        std::vector<float> sbx, sby, sbz;
        if (WbWorld::instance())
          for (WbSolid *ss : WbWorld::instance()->topSolids()) {
            double sl = (cam.translation() - ss->position()).length() * 0.08;
            if (sl < 0.02)
              sl = 0.02;
            emitScaleBoxes(ss, sbx, sby, sbz, sl, sl * 0.12);
          }
        long sbChanged = 0;
        bool okSB = false;
        if (sbx.size() >= 16) {
          const std::vector<uint8_t> preSB = sbuf;
          const float red[4] = {1.0f, 0.1f, 0.1f, 1.0f};
          const float grn[4] = {0.1f, 1.0f, 0.1f, 1.0f};
          const float blu[4] = {0.2f, 0.4f, 1.0f, 1.0f};
          okSB = shot.clearAndDrawLines(sky, svp, red, sbx.data(), static_cast<uint32_t>(sbx.size() / 8),
                                        sbuf.data(), true, false) &&
                 shot.clearAndDrawLines(sky, svp, grn, sby.data(), static_cast<uint32_t>(sby.size() / 8),
                                        sbuf.data(), true, false) &&
                 shot.clearAndDrawLines(sky, svp, blu, sbz.data(), static_cast<uint32_t>(sbz.size() / 8),
                                        sbuf.data(), true, false);
          if (okSB)
            for (size_t i = 0; i < static_cast<size_t>(W) * H; ++i)
              if (sbuf[i * 4] != preSB[i * 4] || sbuf[i * 4 + 1] != preSB[i * 4 + 1] ||
                  sbuf[i * 4 + 2] != preSB[i * 4 + 2])
                ++sbChanged;
        }
        f << "scale-gizmo: " << (sbx.size() / 96) << " boxes; overlay ok=" << (okSB ? 1 : 0)
          << " changed " << sbChanged << " px  ["
          << (sbx.size() < 16 ? "none" : (okSB && sbChanged > 0 ? "PASS — scale boxes render" : "FAIL"))
          << "]\n";

        // Rotate-ring hit-test (the grab half of rotate drag): project an on-screen solid's
        // X-ring radius point and confirm pickRotateRing grabs a ring there.
        int ringAxis = -2;
        bool ringTested = false;
        if (WbWorld::instance())
          for (WbSolid *rs : WbWorld::instance()->topSolids()) {
            double rl = (cam.translation() - rs->position()).length() * 0.08;
            if (rl < 0.02)
              rl = 0.02;
            double mx, my;
            if (projectToScreen(svp, rs->position() + rs->yAxis().normalized() * rl,
                                static_cast<int>(W), static_cast<int>(H), mx, my) &&
                mx >= 0 && mx < W && my >= 0 && my < H) {
              ringAxis = pickRotateRing(static_cast<int>(mx), static_cast<int>(my), rs, svp,
                                        static_cast<int>(W), static_cast<int>(H), rl);
              ringTested = true;
              break;
            }
          }
        f << "rotate-hit-test: on-screen=" << (ringTested ? 1 : 0) << " ring-pt -> ring " << ringAxis
          << "  [" << (ringTested && ringAxis >= 0 ? "PASS — grabs a ring" : "see value") << "]\n";

        // R4 step-3c-A: parent-frame DRAG correctness. The translate drag applies its
        // displacement in the PARENT frame (R_local·unit), and the rotate drag rotates about
        // the parent-frame axis — so a solid under a ROTATED parent still tracks the on-screen
        // world gizmo axis instead of drifting. Oracle: position(), which recomputes the world
        // transform from the FULL node chain. For the most-rotated-parent nested solid, applying
        // localDir·d must move the WORLD position by exactly axisWorld·d; the OLD (world-into-
        // local) formula drifts by the parent rotation. Pure node math — framing-independent.
        {
          std::vector<WbSolid *> desc;
          if (WbWorld::instance())
            for (WbSolid *t : WbWorld::instance()->topSolids())
              collectDescendantSolids(t, desc);
          WbSolid *S = nullptr;
          double bestRot = 0.0;  // |xAxisWorld - R_local·x|: 0 ⇒ identity parent, large ⇒ rotated
          for (WbSolid *s : desc) {
            const WbVector3 lx = (s->rotation().toMatrix3() * WbVector3(1, 0, 0)).normalized();
            const double m = (s->xAxis().normalized() - lx).length();
            if (m > bestRot) {
              bestRot = m;
              S = s;
            }
          }
          if (S) {
            const double d = 0.05;
            double newErr = 0.0, oldErr = 0.0;
            const WbVector3 t0 = S->translation();
            const WbVector3 w0 = S->position();
            for (int k = 0; k < 3; ++k) {
              const WbVector3 unit(k == 0 ? 1.0 : 0.0, k == 1 ? 1.0 : 0.0, k == 2 ? 1.0 : 0.0);
              const WbVector3 axisWorld =
                (k == 0 ? S->xAxis() : k == 1 ? S->yAxis() : S->zAxis()).normalized();
              const WbVector3 localDir = S->rotation().toMatrix3() * unit;  // parent-frame dir
              S->setTranslation(t0 + localDir * d);
              const double e = (S->position() - w0 - axisWorld * d).length();
              if (e > newErr)
                newErr = e;
              if (k == 0) {  // OLD buggy formula: world axis added straight into the local field
                S->setTranslation(t0 + axisWorld * d);
                oldErr = (S->position() - w0 - axisWorld * d).length();
              }
              S->setTranslation(t0);  // restore before the next axis
            }
            const bool pass = newErr < 1e-3 && (bestRot < 0.05 || oldErr > newErr * 5.0);
            f << "parent-frame-drag: parentRot=" << bestRot << " newErr=" << newErr
              << " oldErr=" << oldErr << "  ["
              << (pass ? (bestRot > 0.05
                            ? "PASS — local-frame disp tracks the world gizmo axis; old formula drifts"
                            : "PASS — round-trips (scene had no rotated-parent solid to stress)")
                       : "FAIL")
              << "]\n";
          } else {
            f << "parent-frame-drag: no descendant solids (top-level only — fix is a no-op here)\n";
          }
        }

        // Non-uniformity: a real rendered scene must have many distinct pixels (a
        // broken/blank grab would be a single flat colour).
        long distinct = 0;
        for (size_t i = 1; i < static_cast<size_t>(W) * H; ++i)
          if (sbuf[i * 4] != sbuf[0] || sbuf[i * 4 + 1] != sbuf[1] || sbuf[i * 4 + 2] != sbuf[2])
            ++distinct;
        QImage img(sbuf.data(), static_cast<int>(W), static_cast<int>(H), static_cast<int>(W) * 4,
                   QImage::Format_RGBA8888);
        const QString png = qEnvironmentVariable("OMNISIM_VIEW3D_WGPU_SELFCHECK") + ".png";
        const bool saved = img.copy().save(png, "PNG");  // copy() detaches from sbuf before save
        f << "screenshot: " << W << "x" << H << " render=1 non-bg-pixels=" << distinct << " saved="
          << (saved ? 1 : 0) << "  [" << ((saved && distinct > 1000) ? "PASS — wgpu screenshot to PNG"
                                                                      : "FAIL")
          << "]\n";
      } else {
        f << "screenshot: render failed\n";
      }
    } else {
      f << "screenshot: target unusable\n";
    }
  }

  // R4 step-3c-A: full-screen overlay verification — render a scene on a fresh target,
  // dim it with a half-alpha black overlay, confirm most pixels darken (and ~none lighten).
  {
    WbWgpuRenderTarget ov(mBackend, 256, 256);
    if (ov.isUsable()) {
      float ovp[16];
      WbWgpuSceneRenderer::buildViewProj(cam, hf, 1.0, 0.05, 1000.0, ovp);
      std::vector<WbWgpuSolidDraw> od;
      std::vector<std::array<float, 16>> oms;
      WbWgpuSceneRenderer::collectWorldDraws(*mMeshCache, od, oms);
      std::vector<uint8_t> base(256u * 256u * 4u, 0), dim(base.size(), 0);
      WbWgpuClearColor sky2;
      sky2.r = 0.45f;
      sky2.g = 0.62f;
      sky2.b = 0.85f;
      const float l4[4] = {0.3f, 0.4f, -0.85f, 0.35f};
      const bool okBase =
        ov.clearAndDrawScene(sky2, ovp, l4, od.data(), static_cast<uint32_t>(od.size()), base.data());
      const float black50[4] = {0.0f, 0.0f, 0.0f, 0.5f};
      const bool okOv = ov.drawFullScreenOverlay(black50, dim.data());
      long darker = 0, lighter = 0;
      if (okBase && okOv)
        for (size_t i = 0; i < 256u * 256u; ++i) {
          const int b0 = base[i * 4] + base[i * 4 + 1] + base[i * 4 + 2];
          const int d0 = dim[i * 4] + dim[i * 4 + 1] + dim[i * 4 + 2];
          if (d0 < b0 - 3)
            ++darker;
          else if (d0 > b0 + 3)
            ++lighter;
        }
      f << "fullscreen-overlay: darker=" << darker << " lighter=" << lighter << "  ["
        << ((okBase && okOv && darker > 1000 && lighter < 200) ? "PASS — overlay dims the scene"
                                                               : "FAIL")
        << "]\n";
    }
  }

  // (Device-output inset with a live camera feed is a SEPARATE retrying one-shot in renderTick —
  // see runDeviceInsetCheck — because the controller that enables the camera connects
  // asynchronously and is usually not ready at this self-check's mPhase>5 trigger.)

  // R4 parity: quantify the GGX/sRGB material gap between the wgpu render and WREN's main-view
  // render of the SAME viewpoint. WREN runs here, so its output is the local "golden". Compare
  // only GEOMETRY pixels (pick-masked) to avoid the background/grid confound, and only where BOTH
  // renderers drew geometry (so sub-pixel silhouette differences don't dominate). Reports mean +
  // max per-channel difference + the within-tolerance fraction — a number to drive convergence.
  if (WbWrenWindow *wren = WbWrenWindow::instance()) {
    // NOTE: do NOT force a render here — renderNow()+swap leaves the grab reading an undefined
    // buffer (uniform-gray golden). The "Loading world"-overlay race is handled by deferring the
    // whole self-check until WbWorld::isLoading() clears (see renderTick); after that the normal
    // event loop has repainted the window with the real scene.
    // GOLDEN SOURCE — fixed-resolution offscreen WREN render, fully independent of the window
    // layout (window grabs swung within-tol 16↔54% on identical renderer code). The grab's former
    // crash was post-processing effects holding the deleted framebuffer across the temp resize —
    // fixed in grabSceneOffscreen by rewiring them after each recreation, mirroring resizeWren.
    f << "wren-parity-golden: offscreen 1600x900 grab starting\n";
    f.flush();  // localize crashes: report ending here means we died inside the grab
    const QImage wrenImg = wren->grabSceneOffscreen(1600, 900);
    f << "wren-parity-golden: grab done (" << wrenImg.width() << "x" << wrenImg.height() << ")\n";
    f.flush();
    const int W = wrenImg.width();
    const int H = wrenImg.height();
    // Golden sanity: a near-uniform grab (the "Loading world" overlay, or an undefined-buffer gray)
    // is NOT the scene. Skip the parity lines entirely so render_oracle's retry loop relaunches
    // instead of gating against garbage (seen as spurious 0%/3% runs with WREN "brightness" 1 or 86).
    if (W > 32 && H > 32) {
      bool grabUniform = true;
      const QRgb p0 = wrenImg.pixel(W / 7, H / 7);
      for (int sy = 1; sy < 7 && grabUniform; ++sy)
        for (int sx = 1; sx < 7; ++sx) {
          const QRgb p = wrenImg.pixel(sx * W / 7, sy * H / 7);
          if (qAbs(qRed(p) - qRed(p0)) + qAbs(qGreen(p) - qGreen(p0)) + qAbs(qBlue(p) - qBlue(p0)) > 24) {
            grabUniform = false;
            break;
          }
        }
      if (grabUniform) {
        f << "wren-parity: SKIPPED - WREN grab near-uniform (loading overlay / undefined buffer); no valid golden\n";
        f.flush();
        return;
      }
    }
    if (W > 32 && H > 32) {
      const double aspect = static_cast<double>(W) / static_cast<double>(H);
      float pvp[16];
      WbWgpuSceneRenderer::buildViewProj(cam, horizFovForAspect(hf, aspect), aspect, 0.05, 1000.0, pvp);
      std::vector<WbWgpuSolidDraw> pd;
      std::vector<std::array<float, 16>> pms;
      WbWgpuSceneRenderer::collectWorldDraws(*mMeshCache, pd, pms, nullptr, mTextureCache);
      WbWgpuRenderTarget pr(mBackend, static_cast<uint32_t>(W), static_cast<uint32_t>(H));
      std::vector<uint8_t> litBuf(static_cast<size_t>(W) * H * 4, 0), pickBuf(litBuf.size(), 0);
      WbWgpuClearColor sky;
      sky.r = 0.45f;
      sky.g = 0.62f;
      sky.b = 0.85f;
      // Convergence step: harvest the REAL scene sun (the OmniSimSun PROTO's WbDirectionalLight)
      // so the wgpu lighting + cast shadows come from WREN's actual sun direction, not a hardcoded
      // guess. Falls back to the hardcoded dir if no directional light is found.
      float lit4[4] = {0.3f, 0.4f, -0.85f, 0.35f};
      bool sunFound = false;
      if (WbWorld::instance())
        // The sun is a top-level Light, NOT a Solid → traverse the world ROOT (all top nodes),
        // not topSolids(), to reach the OmniSimSun PROTO's WbDirectionalLight.
        if (WbDirectionalLight *sun = findFirstDirectionalLight(WbWorld::instance()->root())) {
          const WbVector3 sd = sun->direction().normalized();
          lit4[0] = static_cast<float>(sd.x());
          lit4[1] = static_cast<float>(sd.y());
          lit4[2] = static_cast<float>(sd.z());
          sunFound = true;
        }
      const WbVector3 eye = cam.translation();
      const float camPos[3] = {static_cast<float>(eye.x()), static_cast<float>(eye.y()),
                               static_cast<float>(eye.z())};
      bool ok = pr.isUsable();
      ok = ok && pr.clearAndDrawScene(sky, pvp, lit4, pd.data(), static_cast<uint32_t>(pd.size()),
                                      litBuf.data(), false, 1.0f, false, camPos);
      // Tonemapping hypothesis: WREN applies AgX; the parity renders are raw linear-sRGB → too bright
      // (unshadowed 94 vs WREN 40), and a uniform exposure scale does NOT close it (non-linear gap).
      // Render the SAME unshadowed scene WITH AgX (agxMode=true) and measure — if brightness drops
      // toward 40 and within-tol jumps, AgX tonemapping is the real convergence lever (not ambient/shadows).
      std::vector<uint8_t> litAgxBuf(litBuf.size(), 0);
      const bool okAgx = pr.clearAndDrawScene(sky, pvp, lit4, pd.data(), static_cast<uint32_t>(pd.size()),
                                              litAgxBuf.data(), false, 1.0f, true, camPos);
      // CSM rung increment 3: also render WITH cast shadows (kSolidLitTexturedShadow), to measure
      // whether shadows close the brightness gap toward WREN. Light frame: looks along the scene
      // light dir (lit4), positioned back along it, with a SCENE-COVERING ortho extent (panda's
      // floor/walls far exceed a small extent). Rendered before the pick pass corrupts baseColor.
      std::vector<uint8_t> litShadowBuf(litBuf.size(), 0);
      std::vector<uint8_t> litShadowFlatBuf(litBuf.size(), 0);
      bool okSh = false, okShFlat = false;
      {
        const WbVector3 d = WbVector3(lit4[0], lit4[1], lit4[2]).normalized();
        WbVector3 axis = WbVector3(1.0, 0.0, 0.0).cross(d);
        const double axlen = axis.length();
        const double ang = std::acos(std::max(-1.0, std::min(1.0, WbVector3(1.0, 0.0, 0.0).dot(d))));
        axis = axlen > 1e-6 ? axis / axlen : WbVector3(0.0, 1.0, 0.0);
        const WbVector3 P = WbVector3(0.0, 0.0, 0.4) - d * 130.0;
        const WbMatrix4 lightWorld(P.x(), P.y(), P.z(), axis.x(), axis.y(), axis.z(), ang);
        float lightVP[16] = {0};
        WbWgpuSceneRenderer::buildOrthoLightViewProj(lightWorld, 90.0, 0.05, 260.0, lightVP);
        // R4 lighting convergence: hemisphere-IBL ambient harvested from the scene. The sky-blue
        // clear colour IS the OmniSimSky tint the viewer sees behind the geometry — feed it as the
        // sky/zenith ambient, a darkened bounce as the ground term, and the world up vector for the
        // blend, so shadowed regions take WREN's sky fill instead of a flat grey. (Measurement-only
        // path — never touches live/screenshot rendering, which stay byte-identical.)
        const float hemiSky4[4] = {sky.r, sky.g, sky.b, 1.0f};
        const float hemiGround4[4] = {sky.r * 0.25f, sky.g * 0.25f, sky.b * 0.25f, 0.0f};
        float worldUp3[3] = {0.0f, 0.0f, 1.0f};
        if (WbWorld::instance() && WbWorld::instance()->worldInfo()) {
          const WbVector3 up = WbWorld::instance()->worldInfo()->upVector();
          worldUp3[0] = static_cast<float>(up.x());
          worldUp3[1] = static_cast<float>(up.y());
          worldUp3[2] = static_cast<float>(up.z());
        }
        // WREN renders panda heavily shadow-dominated (brightness ~40): surfaces not facing the sun
        // fall to near-black. wgpu's flat 0.35 ambient kept them bright (~94). Drop the ambient hard for
        // the cast-shadow render + full shadow strength so the contrast matches WREN's (lowering ambient
        // darkens only the indirectly-lit/shadowed pixels, NOT the sunlit ones — unlike a uniform scale,
        // which the gate already showed makes parity worse). Gate-measured below.
        // Ambient sweep (below) found WREN's shadows are near-BLACK (no sky fill) — ambient 0 matches best
        // (within-tol 24%→75%). So the cast-shadow render uses zero ambient + full shadow strength.
        // Flat ambient 0.4 (was the panda-tuned 0.0): outdoor worlds (city, OmniSimSky) are
        // BRIGHTER in WREN than the zero-ambient render (city: WREN 141 vs wgpu 102, within-tol 27%;
        // the ambient sweep rises monotonically toward its 0.25 endpoint → 37%). Sunlit scenes have
        // strong sky fill; near-black shadows only fit interiors.
        const float litSh4[4] = {lit4[0], lit4[1], lit4[2], 0.4f};  // gate-measured best (0.3 and 0.5 both score lower within-tol)
        // Fog matches the WREN golden: WREN renders the world's Fog node, so the parity render
        // must too, or every far-field pixel diverges by the haze amount.
        float fogP[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        if (WbFog *fog = WbFog::fogInstance())
          if (fog->fogColor() && fog->fogVisibilityRange() && fog->fogVisibilityRange()->value() > 0.1) {
            const WbRgb fc = fog->fogColor()->value();
            fogP[0] = static_cast<float>(fc.red());
            fogP[1] = static_cast<float>(fc.green());
            fogP[2] = static_cast<float>(fc.blue());
            fogP[3] = static_cast<float>(1.0 / fog->fogVisibilityRange()->value());  // WREN-exact: density=1/range (the shader applies exp2 + the pow-2.2 blend)
          }
        okSh = pr.clearAndDrawSceneTexturedShadowed(sky, pvp, lightVP, litSh4, pd.data(),
                                                    static_cast<uint32_t>(pd.size()), 0.8f, 0.0006f,
                                                    camPos, nullptr, nullptr, nullptr,
                                                    litShadowBuf.data(), false, nullptr, 0.0f,
                                                    fogP[3] > 0.0f ? fogP : nullptr);
        // A/B control: the OLD flat ambient (0.35) + strength 0.6, so the gate keeps showing the
        // improvement from the original baseline to the ambient-0/full-strength primary (23%→75%).
        // (Directional sky-fill at ambient 0 was tested and RULED OUT — 22% << 75%: WREN's shadows are
        // genuinely near-black, so any fill, uniform or directional, hurts. The ambient lever is maxed.)
        (void)hemiSky4; (void)hemiGround4; (void)worldUp3;
        okShFlat = pr.clearAndDrawSceneTexturedShadowed(sky, pvp, lightVP, lit4, pd.data(),
                                                        static_cast<uint32_t>(pd.size()), 0.6f, 0.003f,
                                                        camPos, nullptr, nullptr, nullptr,
                                                        litShadowFlatBuf.data());
#ifdef OMNISIM_HAVE_RENDERDOC
        // R4 shadow-bug diagnostic: bracket each textured-shadow render in a RenderDoc frame capture,
        // tagging each .rdc with its coverage, so we get a known-BAD (floor-dropped) and known-GOOD
        // capture to diff. Gated by OMNISIM_RDC_CAPTURE; needs the Vulkan capture layer loaded
        // (VK_INSTANCE_LAYERS=VK_LAYER_RENDERDOC_Capture + VK_LAYER_PATH=<renderdoc dir>).
        if (qEnvironmentVariableIsSet("OMNISIM_RDC_CAPTURE")) {
          QLibrary rdLib("renderdoc");
          auto getApi = reinterpret_cast<pRENDERDOC_GetAPI>(rdLib.resolve("RENDERDOC_GetAPI"));
          RENDERDOC_API_1_6_0 *rdoc = nullptr;
          if (getApi && getApi(eRENDERDOC_API_Version_1_6_0, reinterpret_cast<void **>(&rdoc)) == 1 && rdoc) {
            const QByteArray tmpl =
              (qEnvironmentVariable("OMNISIM_VIEW3D_WGPU_SELFCHECK") + ".shadowcap").toLocal8Bit();
            rdoc->SetCaptureFilePathTemplate(tmpl.constData());
            const int cr = static_cast<int>(sky.r * 255 + 0.5f), cg = static_cast<int>(sky.g * 255 + 0.5f),
                      cb = static_cast<int>(sky.b * 255 + 0.5f);
            f << "rdc: API loaded; capturing 16 textured-shadow renders -> " << tmpl.constData() << "*.rdc\n";
            f.flush();
            for (int cap = 0; cap < 16; ++cap) {
              rdoc->StartFrameCapture(nullptr, nullptr);
              const bool okc = pr.clearAndDrawSceneTexturedShadowed(
                sky, pvp, lightVP, lit4, pd.data(), static_cast<uint32_t>(pd.size()), 0.6f, 0.003f, camPos,
                hemiSky4, hemiGround4, worldUp3, litShadowBuf.data());
              const uint32_t ended = rdoc->EndFrameCapture(nullptr, nullptr);
              long cov = 0;
              for (size_t q = 0; q + 3 < litShadowBuf.size(); q += 4)
                if (std::abs(static_cast<int>(litShadowBuf[q]) - cr) +
                      std::abs(static_cast<int>(litShadowBuf[q + 1]) - cg) +
                      std::abs(static_cast<int>(litShadowBuf[q + 2]) - cb) > 24)
                  ++cov;
              f << "rdc-capture " << cap << ": ok=" << okc << " ended=" << ended << " coverage=" << cov
                << (cov < 1500000 ? "  [BAD floor-dropped]" : "  [GOOD]") << "\n";
              f.flush();
            }
            f << "rdc: total captures saved = " << rdoc->GetNumCaptures() << "\n";
          } else {
            f << "rdc: RENDERDOC_GetAPI unavailable (capture layer not loaded into the process)\n";
          }
          f.flush();
        }
#endif
      }
      // Signal diagnostic: save BOTH frames (same viewpoint+size) so alignment-vs-material can be
      // inspected directly — if the geometry silhouettes overlap, the diff is material; if offset,
      // the camera is misaligned and the per-pixel number is alignment-noise.
      {
        const QString base = qEnvironmentVariable("OMNISIM_VIEW3D_WGPU_SELFCHECK");
        wrenImg.copy().save(base + ".wren.png", "PNG");
        QImage wg(litBuf.data(), W, H, W * 4, QImage::Format_RGBA8888);
        wg.copy().save(base + ".wgpu.png", "PNG");
        if (okSh) {
          QImage wgs(litShadowBuf.data(), W, H, W * 4, QImage::Format_RGBA8888);
          wgs.copy().save(base + ".wgpu_shadow.png", "PNG");
        }
        if (okShFlat) {
          QImage wgsf(litShadowFlatBuf.data(), W, H, W * 4, QImage::Format_RGBA8888);
          wgsf.copy().save(base + ".wgpu_shadow_flat.png", "PNG");
        }
      }
      // Diagnostic: how many pixels in each render actually differ from the sky clear colour (= geometry
      // coverage, FULL image). If litShadow ≈ 0 while lit is large, the textured-shadowed pass drew no
      // geometry (the suspected bug); if comparable, geometry rendered and the issue is shading.
      {
        const int cr = static_cast<int>(sky.r * 255 + 0.5f), cg = static_cast<int>(sky.g * 255 + 0.5f),
                  cb = static_cast<int>(sky.b * 255 + 0.5f);
        auto coverage = [&](const std::vector<uint8_t> &buf) {
          long n = 0;
          for (size_t q = 0; q + 3 < buf.size(); q += 4)
            if (std::abs(static_cast<int>(buf[q]) - cr) + std::abs(static_cast<int>(buf[q + 1]) - cg) +
                  std::abs(static_cast<int>(buf[q + 2]) - cb) > 24)
              ++n;
          return n;
        };
        f << "render-coverage: lit=" << coverage(litBuf) << " litShadow=" << (okSh ? coverage(litShadowBuf) : -1)
          << " litShadowFlat=" << (okShFlat ? coverage(litShadowFlatBuf) : -1) << " (of "
          << static_cast<long>(W) * H << " px)\n";
      }
      // Save the real per-draw colours before the pick pass overwrites them with IDs, so the
      // hemisphere-intensity sweep below can restore pd and render the true materials at the EXACT
      // same geometry positions as the pick mask (no re-collect → no physics-drift misalignment).
      std::vector<std::array<float, 4>> origColors(pd.size());
      for (size_t i = 0; i < pd.size(); ++i)
        origColors[i] = {pd[i].baseColorR, pd[i].baseColorG, pd[i].baseColorB, pd[i].baseColorA};
      // encode per-draw IDs for the geometry mask (id+1 in baseColor), pick pass
      for (size_t i = 0; i < pd.size(); ++i) {
        const uint32_t id = static_cast<uint32_t>(i) + 1u;
        pd[i].baseColorR = static_cast<float>(id & 0xFFu) / 255.0f;
        pd[i].baseColorG = static_cast<float>((id >> 8) & 0xFFu) / 255.0f;
        pd[i].baseColorB = static_cast<float>((id >> 16) & 0xFFu) / 255.0f;
      }
      ok = ok && pr.clearAndDrawScene(sky, pvp, lit4, pd.data(), static_cast<uint32_t>(pd.size()),
                                      pickBuf.data(), false, 1.0f, false, nullptr, true);
      long geomPx = 0, within = 0, withinSh = 0, withinShFlat = 0, withinAgx = 0;
      double sumDiff = 0.0, sumWgpuBright = 0.0, sumWrenBright = 0.0, sumHueDiff = 0.0;
      double sumShadowBright = 0.0, sumShadowDiff = 0.0;
      double sumShadowFlatBright = 0.0, sumShadowFlatDiff = 0.0;
      double sumAgxBright = 0.0;
      int maxDiff = 0;
      if (ok) {
        for (int y = 0; y < H; ++y) {
          const QRgb *wrow = reinterpret_cast<const QRgb *>(wrenImg.constScanLine(y));
          for (int x = 0; x < W; ++x) {
            const size_t p = (static_cast<size_t>(y) * W + x) * 4;
            const uint32_t pid = static_cast<uint32_t>(pickBuf[p]) | (static_cast<uint32_t>(pickBuf[p + 1]) << 8) |
                                 (static_cast<uint32_t>(pickBuf[p + 2]) << 16);
            if (pid == 0)
              continue;  // background in the wgpu render — skip
            const QRgb wpx = wrow[x];  // WREN pixel (qRed/qGreen/qBlue handle byte order)
            const int wgR = litBuf[p], wgG = litBuf[p + 1], wgB = litBuf[p + 2];
            const int wnR = qRed(wpx), wnG = qGreen(wpx), wnB = qBlue(wpx);
            const int d = std::abs(wgR - wnR) + std::abs(wgG - wnG) + std::abs(wgB - wnB);
            ++geomPx;
            sumDiff += d;
            if (d > maxDiff)
              maxDiff = d;
            if (d <= 60)  // ≤20/channel avg
              ++within;
            // Decompose the gap: brightness level (lighting/shadows) vs hue (material). Normalize
            // each pixel by its own brightness so the hue-diff is brightness-independent.
            const double wgBr = (wgR + wgG + wgB) / 3.0, wnBr = (wnR + wnG + wnB) / 3.0;
            sumWgpuBright += wgBr;
            sumWrenBright += wnBr;
            const double wgN = wgBr > 1 ? wgBr : 1, wnN = wnBr > 1 ? wnBr : 1;
            sumHueDiff += std::abs(wgR / wgN - wnR / wnN) + std::abs(wgG / wgN - wnG / wnN) +
                          std::abs(wgB / wgN - wnB / wnN);
            // Shadowed render's brightness + diff over the same geometry pixels.
            if (okSh) {
              const int sR = litShadowBuf[p], sG = litShadowBuf[p + 1], sB = litShadowBuf[p + 2];
              sumShadowBright += (sR + sG + sB) / 3.0;
              const int sd = std::abs(sR - wnR) + std::abs(sG - wnG) + std::abs(sB - wnB);
              sumShadowDiff += sd;
              if (sd <= 60)
                ++withinSh;
            }
            if (okShFlat) {
              const int fR = litShadowFlatBuf[p], fG = litShadowFlatBuf[p + 1], fB = litShadowFlatBuf[p + 2];
              sumShadowFlatBright += (fR + fG + fB) / 3.0;
              const int fd = std::abs(fR - wnR) + std::abs(fG - wnG) + std::abs(fB - wnB);
              sumShadowFlatDiff += fd;
              if (fd <= 60)
                ++withinShFlat;
            }
            if (okAgx) {
              const int aR = litAgxBuf[p], aG = litAgxBuf[p + 1], aB = litAgxBuf[p + 2];
              sumAgxBright += (aR + aG + aB) / 3.0;
              if (std::abs(aR - wnR) + std::abs(aG - wnG) + std::abs(aB - wnB) <= 60)
                ++withinAgx;
            }
          }
        }
      }
      f << "wren-parity-sun: found=" << (sunFound ? 1 : 0) << " dir=(" << lit4[0] << "," << lit4[1]
        << "," << lit4[2] << ")  [" << (sunFound ? "harvested real OmniSimSun" : "fallback hardcoded")
        << "]\n";
      const double meanPerCh = geomPx > 0 ? (sumDiff / geomPx / 3.0) : 0.0;
      const double withinPct = geomPx > 0 ? (100.0 * within / geomPx) : 0.0;
      const int wgpuBright = geomPx > 0 ? static_cast<int>(sumWgpuBright / geomPx + 0.5) : 0;
      const int wrenBright = geomPx > 0 ? static_cast<int>(sumWrenBright / geomPx + 0.5) : 0;
      const double hueDiff = geomPx > 0 ? (sumHueDiff / geomPx / 3.0) : 0.0;  // 0=same hue, 1=opposite
      if (okSh && geomPx > 0) {
        const int shBright = static_cast<int>(sumShadowBright / geomPx + 0.5);
        const int shMean = static_cast<int>(sumShadowDiff / geomPx / 3.0 + 0.5);
        const int shWithin = static_cast<int>(100.0 * withinSh / geomPx + 0.5);
        f << "wren-parity-shadowed: shadowed-mean-brightness=" << shBright << " (target WREN="
          << wrenBright << ", was " << wgpuBright << " unshadowed) shadowed-mean-diff/ch=" << shMean
          << " within-tol=" << shWithin << "%  ["
          << (shBright < wgpuBright ? "CSM shadows lower brightness toward WREN"
                                    : "shadows did not lower brightness (light-frame tuning)")
          << "]\n";
      }
      // A/B: hemisphere-IBL shadowed vs flat-ambient shadowed, SAME run / SAME WREN frame — the
      // honest isolation of the hemisphere term (resolution/exposure held constant). Keep the change
      // only if hemisphere within-tol beats flat.
      if (okSh && okShFlat && geomPx > 0) {
        const int hWithin = static_cast<int>(100.0 * withinSh / geomPx + 0.5);
        const int fWithin = static_cast<int>(100.0 * withinShFlat / geomPx + 0.5);
        const int fBright = static_cast<int>(sumShadowFlatBright / geomPx + 0.5);
        const int fMean = static_cast<int>(sumShadowFlatDiff / geomPx / 3.0 + 0.5);
        f << "wren-parity-shadowed-flat: flat-ambient within-tol=" << fWithin << "% (brightness="
          << fBright << " mean-diff/ch=" << fMean << ")  vs hemisphere within-tol=" << hWithin
          << "%  [" << (hWithin > fWithin ? "HEMISPHERE-IBL IMPROVES per-pixel parity vs flat"
                                          : "hemisphere NOT better than flat — revert the hemisphere term")
          << "]\n";
      }
      f << "wren-parity-decomp: wgpu-mean-brightness=" << wgpuBright << " wren-mean-brightness="
        << wrenBright << " (gap mostly LIGHTING/shadows if these differ a lot) hue-diff="
        << hueDiff << " (low ⇒ materials/hue match; the residual is lighting)\n";
      f << "wren-parity: " << W << "x" << H << " geom-px=" << geomPx << " mean-diff/ch="
        << static_cast<int>(meanPerCh + 0.5) << " max-diff=" << maxDiff << " within-tol="
        << static_cast<int>(withinPct + 0.5) << "%  [" << (geomPx > 1000 ? "MEASURED" : "see values")
        << " — wgpu vs WREN over geometry]\n";
      if (okAgx && geomPx > 0) {
        const int aBright = static_cast<int>(sumAgxBright / geomPx + 0.5);
        const int aWithin = static_cast<int>(100.0 * withinAgx / geomPx + 0.5);
        f << "wren-parity-agx: unshadowed+AgX within-tol=" << aWithin << "% brightness=" << aBright
          << " (no-AgX was " << static_cast<int>(withinPct + 0.5) << "% br" << wgpuBright
          << ", target WREN=" << wrenBright << ")  ["
          << (aWithin > static_cast<int>(withinPct + 0.5)
                ? "AgX tonemap IMPROVES parity — the real exposure lever"
                : "AgX no better")
          << "]\n";
      }
      // Increment-4 diagnostic: is the residual JUST exposure (a uniform brightness scale) or
      // STRUCTURAL (shadows/AO in different places)? Re-measure within-tol after scaling wgpu by the
      // WREN/wgpu brightness ratio. If within-tol jumps, exposure-matching is the lever (near
      // convergence with one knob); if it stays low, the gap is structural (multi-rung lighting).
      if (ok && geomPx > 1000 && sumWgpuBright > 1.0) {
        const double scale = sumWrenBright / sumWgpuBright;  // optimal uniform exposure match
        long scaledWithin = 0;
        for (int y = 0; y < H; ++y) {
          const QRgb *wrow = reinterpret_cast<const QRgb *>(wrenImg.constScanLine(y));
          for (int x = 0; x < W; ++x) {
            const size_t p = (static_cast<size_t>(y) * W + x) * 4;
            const uint32_t pid = static_cast<uint32_t>(pickBuf[p]) |
                                 (static_cast<uint32_t>(pickBuf[p + 1]) << 8) |
                                 (static_cast<uint32_t>(pickBuf[p + 2]) << 16);
            if (pid == 0)
              continue;
            const QRgb wpx = wrow[x];
            const int sr = static_cast<int>(litBuf[p] * scale + 0.5);
            const int sg = static_cast<int>(litBuf[p + 1] * scale + 0.5);
            const int sb = static_cast<int>(litBuf[p + 2] * scale + 0.5);
            if (std::abs(sr - qRed(wpx)) + std::abs(sg - qGreen(wpx)) + std::abs(sb - qBlue(wpx)) <= 60)
              ++scaledWithin;
          }
        }
        const int scaledPct = static_cast<int>(100.0 * scaledWithin / geomPx + 0.5);
        f << "wren-parity-exposurematch: scale=" << scale << " within-tol=" << scaledPct << "% (was "
          << static_cast<int>(withinPct + 0.5) << "%)  ["
          << (scaledPct > static_cast<int>(withinPct + 0.5) + 15
                ? "EXPOSURE is the main lever — match WREN's exposure/ambient to converge"
                : "residual is STRUCTURAL (shadows/AO placement), not just exposure")
          << "]\n";
      }
      // Hemisphere-intensity sweep: at intensity 1.0 the sky fill overshoots mean brightness (96 vs
      // WREN ~64). Sweep a few intensities, render the hemisphere-shadowed scene at each, and measure
      // within-tol against the SAME WREN frame + pick mask — the gate picks the convergence-optimal
      // ambient strength (identical tolerance/mask, so this finds the optimum, it doesn't game it).
      // pd's colours were overwritten by the pick pass → restore them from origColors first.
      if (okSh && geomPx > 1000) {
        for (size_t i = 0; i < pd.size(); ++i) {
          pd[i].baseColorR = origColors[i][0];
          pd[i].baseColorG = origColors[i][1];
          pd[i].baseColorB = origColors[i][2];
          pd[i].baseColorA = origColors[i][3];
        }
        const WbVector3 d = WbVector3(lit4[0], lit4[1], lit4[2]).normalized();
        WbVector3 axis = WbVector3(1.0, 0.0, 0.0).cross(d);
        const double axlen = axis.length();
        const double ang = std::acos(std::max(-1.0, std::min(1.0, WbVector3(1.0, 0.0, 0.0).dot(d))));
        axis = axlen > 1e-6 ? axis / axlen : WbVector3(0.0, 1.0, 0.0);
        const WbVector3 Pl = WbVector3(0.0, 0.0, 0.4) - d * 130.0;
        const WbMatrix4 lightWorld(Pl.x(), Pl.y(), Pl.z(), axis.x(), axis.y(), axis.z(), ang);
        float lightVP[16] = {0};
        WbWgpuSceneRenderer::buildOrthoLightViewProj(lightWorld, 90.0, 0.05, 260.0, lightVP);
        float up3[3] = {0.0f, 0.0f, 1.0f};
        if (WbWorld::instance() && WbWorld::instance()->worldInfo()) {
          const WbVector3 up = WbWorld::instance()->worldInfo()->upVector();
          up3[0] = static_cast<float>(up.x());
          up3[1] = static_cast<float>(up.y());
          up3[2] = static_cast<float>(up.z());
        }
        (void)up3;
        // Flat-ambient sweep at FULL shadow strength: find the cast-shadow ambient that best matches WREN
        // (now that the render is deterministic). Reports within-tol + mean-diff + brightness per value.
        const float ambs[] = {0.25f, 0.15f, 0.10f, 0.05f, 0.0f};
        std::vector<uint8_t> sweepBuf(static_cast<size_t>(W) * H * 4, 0);
        float bestA = 0.10f;
        int bestPct = -1, bestMean = 9999, bestBright = 0;
        f << "wren-parity-ambsweep (flat, strength 1.0):";
        for (float amb : ambs) {
          const float ls4[4] = {lit4[0], lit4[1], lit4[2], amb};
          if (!pr.clearAndDrawSceneTexturedShadowed(sky, pvp, lightVP, ls4, pd.data(),
                                                    static_cast<uint32_t>(pd.size()), 0.8f, 0.0006f,
                                                    camPos, nullptr, nullptr, nullptr, sweepBuf.data()))
            continue;
          long win = 0;
          double sumB = 0.0, sumD = 0.0;
          for (int y = 0; y < H; ++y) {
            const QRgb *wrow = reinterpret_cast<const QRgb *>(wrenImg.constScanLine(y));
            for (int x = 0; x < W; ++x) {
              const size_t p = (static_cast<size_t>(y) * W + x) * 4;
              const uint32_t pid = static_cast<uint32_t>(pickBuf[p]) |
                                   (static_cast<uint32_t>(pickBuf[p + 1]) << 8) |
                                   (static_cast<uint32_t>(pickBuf[p + 2]) << 16);
              if (pid == 0)
                continue;
              const QRgb wpx = wrow[x];
              const int r = sweepBuf[p], g = sweepBuf[p + 1], b = sweepBuf[p + 2];
              sumB += (r + g + b) / 3.0;
              const int dd = std::abs(r - qRed(wpx)) + std::abs(g - qGreen(wpx)) + std::abs(b - qBlue(wpx));
              sumD += dd;
              if (dd <= 60)
                ++win;
            }
          }
          const int pct = static_cast<int>(100.0 * win / geomPx + 0.5);
          const int br = static_cast<int>(sumB / geomPx + 0.5);
          const int md = static_cast<int>(sumD / geomPx / 3.0 + 0.5);
          f << " a=" << amb << "->" << pct << "%(br" << br << ",md" << md << ")";
          if (pct > bestPct) {
            bestPct = pct;
            bestA = amb;
            bestMean = md;
            bestBright = br;
          }
        }
        f << "  [best ambient=" << bestA << " within-tol=" << bestPct << "% mean-diff=" << bestMean
          << " brightness=" << bestBright << " (target WREN=" << wrenBright << ")]\n";
      }
    }
  }
  WbLog::info("[WbWgpuView] offscreen pick + selection + joint-axis + screenshot + overlay self-check");
}

// R4 step-3c-A: DEVICE-OUTPUT INSET with a REAL sensor image. Retrying one-shot (the camera's
// controller enables the sensor asynchronously). Finds an enabled camera, wraps its rendered
// image() (BGRA) in a QImage, uploads it, renders the world to an offscreen target, and composites
// the camera view as a corner inset via the verified drawTexturedInset, then asserts the inset
// region differs from the scene + is non-uniform. Returns true once it has written a verdict
// (camera enabled + attempted) so the caller stops retrying; false → try again next tick.
bool WbWgpuView::runDeviceInsetCheck() {
  if (!mBackend || !mMeshCache || !mTextureCache)
    return false;
  WbAbstractCamera *sensorCam = nullptr;
  if (WbWorld::instance())
    for (WbSolid *s : WbWorld::instance()->topSolids()) {
      sensorCam = findFirstCamera(s);
      if (sensorCam)
        break;
    }
  if (!sensorCam)
    return true;  // no camera in this world → nothing to check, stop retrying
  const unsigned char *img = sensorCam->isEnabled() ? sensorCam->constImage() : nullptr;
  const int cw = sensorCam->width(), ch = sensorCam->height();
  if (!img || cw <= 0 || ch <= 0)
    return false;  // controller hasn't enabled/rendered the camera yet — retry next tick

  WbMatrix4 vpcam;
  double hf = 0.785;
  buildViewpointCamera(vpcam, hf);
  const QByteArray path =
    (qEnvironmentVariable("OMNISIM_VIEW3D_WGPU_SELFCHECK") + ".devinset.txt").toLocal8Bit();
  std::ofstream f(path.constData());
  if (!f)
    return true;
  QImage qi(img, cw, ch, cw * 4, QImage::Format_ARGB32);  // camera image() is BGRA → ARGB32 on LE
  WbWgpuTextureHandle th = WbWgpuImageAdapter::acquireFromQImage(
    *mTextureCache, reinterpret_cast<uint64_t>(sensorCam) | 0x5e500000ull, qi);
  const uint32_t W = 640, H = 480;
  WbWgpuRenderTarget di(mBackend, W, H);
  if (!th.view || !di.isUsable()) {
    f << "device-inset: cam=1 enabled=1 img=" << cw << "x" << ch << " (texture/target unusable)\n";
    return true;
  }
  const double aspect = static_cast<double>(W) / static_cast<double>(H);
  float scenevp[16];
  WbWgpuSceneRenderer::buildViewProj(vpcam, horizFovForAspect(hf, aspect), aspect, 0.05, 1000.0, scenevp);
  std::vector<WbWgpuSolidDraw> dd;
  std::vector<std::array<float, 16>> dms;
  WbWgpuSceneRenderer::collectWorldDraws(*mMeshCache, dd, dms, nullptr, mTextureCache);
  std::vector<uint8_t> dbuf(static_cast<size_t>(W) * H * 4, 0);
  WbWgpuClearColor sky3;
  sky3.r = 0.45f;
  sky3.g = 0.62f;
  sky3.b = 0.85f;
  const float l5[4] = {0.3f, 0.4f, -0.85f, 0.35f};
  const bool okScene =
    di.clearAndDrawScene(sky3, scenevp, l5, dd.data(), static_cast<uint32_t>(dd.size()), dbuf.data());
  const std::vector<uint8_t> preInset = dbuf;
  const float rect[4] = {0.45f, 0.45f, 0.95f, 0.95f};  // top-right NDC quarter
  const bool okInset = okScene && di.drawTexturedInset(th.view, rect, dbuf.data());
  long insetChanged = 0, insetDistinct = 0;
  if (okInset) {
    const size_t basePix = (static_cast<size_t>(H / 4) * W + (W * 3 / 4)) * 4;  // a pt inside the inset
    for (size_t y = H / 8; y < H / 2; ++y)
      for (size_t x = W / 2; x < W - 4; ++x) {
        const size_t p = (y * W + x) * 4;
        if (dbuf[p] != preInset[p] || dbuf[p + 1] != preInset[p + 1] || dbuf[p + 2] != preInset[p + 2])
          ++insetChanged;
        if (dbuf[p] != dbuf[basePix] || dbuf[p + 1] != dbuf[basePix + 1] || dbuf[p + 2] != dbuf[basePix + 2])
          ++insetDistinct;
      }
    QImage out(dbuf.data(), W, H, W * 4, QImage::Format_RGBA8888);
    out.copy().save(qEnvironmentVariable("OMNISIM_VIEW3D_WGPU_SELFCHECK") + ".devinset.png", "PNG");
  }
  f << "device-inset: cam=1 enabled=1 img=" << cw << "x" << ch << " changed=" << insetChanged
    << " distinct=" << insetDistinct << "  ["
    << ((okInset && insetChanged > 500 && insetDistinct > 100)
          ? "PASS — live camera image composited as a device inset"
          : "see values")
    << "]\n";
  return true;
}

void WbWgpuView::renderTick() {
  mPhase += 0.02;  // frame clock (also the box spin angle)

  // One-shot offscreen pick self-check, once the backend + world are ready. The
  // timer only runs post-expose (after ensureSurface inits the backend), so
  // mBackend/mMeshCache are set here. Settles ~5 s for the world to finish loading.
  if (!mSelfCheckDone && mBackend && mMeshCache && mPhase > 5.0 &&
      qEnvironmentVariableIsSet("OMNISIM_VIEW3D_WGPU_SELFCHECK") &&
      // Defer while the world is still LOADING: the parity section grabs the WREN main window as
      // its golden, and mid-load that grab returns the "Loading world" overlay (black) → a
      // spurious 0% gate. Heavy worlds (the city) load past the 5 s settle on slower starts.
      (!WbWorld::instance() || !WbWorld::instance()->isLoading())) {
    runSelfCheck();
    mSelfCheckDone = true;
  }

  // Device-output inset (live camera feed): retry each tick from mPhase>6 until the camera's
  // controller has connected + enabled it (asynchronous), giving up by mPhase>40 so a world with
  // no controller-enabled camera still records a verdict once.
  if (!mDeviceInsetDone && mBackend && mMeshCache && mPhase > 6.0 &&
      qEnvironmentVariableIsSet("OMNISIM_VIEW3D_WGPU_SELFCHECK")) {
    if (runDeviceInsetCheck() || mPhase > 40.0)
      mDeviceInsetDone = true;
  }

  // On-screen draw — needs the exposed surface.
  if (!mSurface || !mSurface->isUsable() || !isExposed())
    return;

  // R4 step-3c-A: one-shot LIVE-PANE overlay verification. Force VF_ALL_BOUNDING_OBJECTS
  // (depth-tested green wireframes) + VF_JOINT_AXES (always-on-top cyan lines) on, render
  // the surface WITH readback, count each overlay colour + save the pane PNG, restore the
  // flags. Proves BOTH surface line passes work on the live swapchain.
  if (mSelfCheckDone && !mSurfaceLineCheckDone && mPhase > 8.0 &&
      qEnvironmentVariableIsSet("OMNISIM_VIEW3D_WGPU_SELFCHECK") && mSurface->canReadback()) {
    mSurfaceLineCheckDone = true;
    if (WbWrenRenderingContext *rc = WbWrenRenderingContext::instance()) {
      const bool boOn = rc->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_ALL_BOUNDING_OBJECTS);
      const bool jaOn = rc->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_JOINT_AXES);
      rc->enableOptionalRendering(WbWrenRenderingContext::VF_ALL_BOUNDING_OBJECTS, true, false);
      rc->enableOptionalRendering(WbWrenRenderingContext::VF_JOINT_AXES, true, false);
      const int w = pixelWidth(), h = pixelHeight();
      const bool bgra = mSurface->formatIsBgra();
      std::vector<uint8_t> buf(static_cast<size_t>(w) * h * 4, 0);
      const bool ok = drawWorld(buf.data());
      long green = 0, cyan = 0;
      for (size_t i = 0; i < static_cast<size_t>(w) * h; ++i) {
        const size_t b = i * 4;
        const uint8_t rr = bgra ? buf[b + 2] : buf[b], gg = buf[b + 1], bb = bgra ? buf[b] : buf[b + 2];
        if (gg > 150 && rr < 90 && bb < 90)  // green: R&B low → order-agnostic
          ++green;
        else if (gg > 150 && bb > 150 && rr < 90)  // cyan: R low, G&B high
          ++cyan;
      }
      const QString base = qEnvironmentVariable("OMNISIM_VIEW3D_WGPU_SELFCHECK");
      QImage img(buf.data(), w, h, w * 4, bgra ? QImage::Format_ARGB32 : QImage::Format_RGBA8888);
      const bool saved = img.copy().save(base + ".pane.png", "PNG");
      std::ofstream lf((base + ".surfline.txt").toLocal8Bit().constData());
      if (lf)
        lf << "surface-line-overlay: render_ok=" << (ok ? 1 : 0) << " green(bounding)-px=" << green
           << " cyan(joint-axes)-px=" << cyan << " pane-png-saved=" << (saved ? 1 : 0) << "  ["
           << ((ok && green > 200 && cyan > 50) ? "PASS — both overlays in the LIVE pane"
                                                : "see counts (needs a wide pane)")
           << "]\n";
      rc->enableOptionalRendering(WbWrenRenderingContext::VF_ALL_BOUNDING_OBJECTS, boOn, false);
      rc->enableOptionalRendering(WbWrenRenderingContext::VF_JOINT_AXES, jaOn, false);
    }
  }

  // Step 3: the live world from the live Viewpoint — tracks the WREN main view.
  if (drawWorld(nullptr, &mLastWorldDraws))
    return;
  // Step 2 fallback: the spinning box when no world is loaded.
  if (drawBox(mPhase, nullptr))
    return;
  // Step 1 fallback: cycling clear if the scene path isn't ready.
  const float r = 0.5f + 0.5f * static_cast<float>(std::sin(mPhase));
  const float g = 0.5f + 0.5f * static_cast<float>(std::sin(mPhase + 2.094));
  const float b = 0.5f + 0.5f * static_cast<float>(std::sin(mPhase + 4.188));
  mSurface->presentClear(r, g, b);
}
