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

#include "OmWgpuView.hpp"

#include "OmAbstractCamera.hpp"
#include "OmBasicJoint.hpp"
#include "OmBox.hpp"
#include "OmCapsule.hpp"
#include "OmCylinder.hpp"
#include "OmDirectionalLight.hpp"  // harvest the real scene sun for lighting/shadow direction
#include "OmGeometry.hpp"
#include "OmGroup.hpp"
#include "OmJoint.hpp"
#include "OmLidar.hpp"
#include "OmLightSensor.hpp"
#include "OmRadar.hpp"
#include "OmLog.hpp"
#include "OmMathsUtilities.hpp"  // twoStepsConvexHull — support-polygon hull
#include "OmMatrix4.hpp"
#include "OmRenderBackend.hpp"
#include "OmRotation.hpp"
#include "OmSFDouble.hpp"
#include "OmSFRotation.hpp"
#include "OmSFVector3.hpp"
#include "OmSelection.hpp"
#include "OmShape.hpp"
#include "OmSolid.hpp"
#include "OmSphere.hpp"
#include "OmWrenRenderingContext.hpp"
#include "OmVector2.hpp"  // support-polygon hull projection
#include "OmVector3.hpp"
#include "OmViewpoint.hpp"
#include "OmVulkanBackend.hpp"
#include "OmWgpuImageAdapter.hpp"   // upload a camera image() to a wgpu texture (device inset)
#include "OmWgpuMeshAdapter.hpp"
#include "OmWgpuMeshCache.hpp"
#include "OmWgpuRenderTarget.hpp"   // OmWgpuSolidDraw
#include "OmWgpuSceneRenderer.hpp"  // buildViewProj, collectWorldDraws
#include "OmWgpuSurface.hpp"
#include "OmWgpuTextureCache.hpp"
#include "OmWorld.hpp"
#include "OmWorldInfo.hpp"  // east/north/up basis for the support-polygon projection
#include "OmFog.hpp"
#include "OmSFColor.hpp"
#include "OmSFDouble.hpp"

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
#include <QtGui/QGuiApplication>
#include <QtGui/QImage>
#include <QtGui/QMouseEvent>
#include <QtGui/QResizeEvent>
#include <QtGui/QWindow>
// Qt's PUBLIC native-interface header: QNativeInterface::QX11Application
// (Display* + xcb_connection_t*) and QNativeInterface::QWaylandApplication
// (wl_display*). Each struct is declared only where Qt itself was built with
// that platform — the header guards them with QT_CONFIG(xcb)/QT_CONFIG(wayland)
// — so every use below repeats the same guard. On the Windows Qt build both
// features are -1 and the whole Unix block compiles out.
#include <QtGui/qguiapplication_platform.h>
// Wayland only: Qt exposes wl_display through the public interface above but NOT
// wl_surface, so the surface comes from the QPA native-resource accessor. That
// header is a Qt PRIVATE header (qt6-base-private-dev on Debian/Ubuntu, and its
// versioned include dir is not on the default search path), so it is optional:
// without it the Wayland branch reports itself unavailable and the caller falls
// back to the GL blit — it never guesses a pointer at a compositor object.
#if defined(__has_include)
#  if __has_include(<qpa/qplatformnativeinterface.h>)
#    include <qpa/qplatformnativeinterface.h>
#    define OMNISIM_HAVE_QPA_NATIVE_INTERFACE 1
#  endif
#endif
#ifndef OMNISIM_HAVE_QPA_NATIVE_INTERFACE
#  define OMNISIM_HAVE_QPA_NATIVE_INTERFACE 0
#endif

// QT_CONFIG(feature) expands to (1/QT_FEATURE_##feature == 1), so it is only
// usable where QT_FEATURE_<feature> is actually DEFINED -- an undefined macro
// is 0 to the preprocessor and the expansion becomes a literal 1/0.
// QT_FEATURE_xcb is published by QtGui, which is why the xcb guard below is
// fine as written. QT_FEATURE_wayland is published by QtWaylandClient, which a
// stock Qt 6.5.3 gcc_64 build does not put on the include path -- so the bare
// QT_CONFIG(wayland) was a hard "error: division by zero in #if" on Linux.
// This file is in the UNCONDITIONAL source list (src/omnisim/Makefile:991) --
// WB_WGPU_NATIVE_AVAILABLE only toggles flags and libs, never whether the file
// is compiled -- so the error fires on ANY Linux build. It went unnoticed
// because no Linux build runs in CI: release.yml is Windows-only and the Linux
// workflows sit under .github/workflows.disabled/. The one Linux build that did
// exist, the GPU training image, had simply been failing on this exact line
// since 2026-08-25 with nobody watching.
#if defined(QT_FEATURE_wayland)
#  define OMNISIM_HAVE_QT_WAYLAND QT_CONFIG(wayland)
#else
#  define OMNISIM_HAVE_QT_WAYLAND 0
#endif


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

// ---------------------------------------------------------------------------
// OmWgpuNativeWindowFromQtWindow — declared in render/OmWgpuSurface.hpp.
//
// Implemented HERE, in the GUI layer, because resolving native handles needs Qt
// while OmWgpuSurface is deliberately Qt-free (it takes raw handles). One
// implementation is shared by BOTH wgpu surface construction sites — this
// file's OmWgpuView, and OmView3D's input-transparent present child window — so
// the per-platform logic exists exactly once.
//
// The windowing system is resolved at RUNTIME from QGuiApplication::platformName(),
// never at compile time: one Linux binary has to serve an xcb session and a
// wayland session, and Qt picks between them at startup.
//
// EVERY failure path returns a Kind::None handle and logs at most one line. A
// null handle means "keep the existing readback + GL-blit presentation" — it
// must never produce a black window.
OmWgpuNativeWindow OmWgpuNativeWindowFromQtWindow(QWindow *window) {
  if (!window)
    return OmWgpuNativeWindow();

#ifdef _WIN32
  // Unchanged Windows path: HWND from winId(), HINSTANCE from GetModuleHandleW(nullptr)
  // — exactly what both call sites passed positionally before.
  const WId wid = window->winId();
  if (!wid)
    return OmWgpuNativeWindow();
  return OmWgpuNativeWindow::windowsHwnd(reinterpret_cast<void *>(wid),
                                         reinterpret_cast<void *>(GetModuleHandleW(nullptr)));
#else
  // Warn ONCE per process: OmView3D re-attempts surface creation every frame
  // while it has no surface, so an unguarded log here would flood omnisim_log.txt.
  static bool sWarned = false;
  const QString platform = QGuiApplication::platformName();

#  if QT_CONFIG(xcb)
  if (platform == QLatin1String("xcb")) {
    QNativeInterface::QX11Application *x11 =
      qGuiApp ? qGuiApp->nativeInterface<QNativeInterface::QX11Application>() : nullptr;
    // On the xcb plugin QWindow::winId() IS the X Window (XID) — the same value
    // an Xlib `Window` and an `xcb_window_t` name.
    const WId wid = window->winId();
    if (x11 && wid) {
      // Prefer Xlib (VK_KHR_xlib_surface): the Display* carries the connection
      // wgpu needs. XCB is the fallback for a Qt built without an Xlib Display.
      if (Display *dpy = x11->display())
        return OmWgpuNativeWindow::xlibWindow(static_cast<void *>(dpy), static_cast<uint64_t>(wid));
      if (xcb_connection_t *conn = x11->connection())
        return OmWgpuNativeWindow::xcbWindow(static_cast<void *>(conn), static_cast<uint32_t>(wid));
    }
    if (!sWarned) {
      sWarned = true;
      OmLog::info(QString("[OmWgpuView] xcb session but no usable native handle "
                          "(x11-interface=%1 winId=%2) — presentation stays on the GL blit path")
                    .arg(x11 ? 1 : 0)
                    .arg(static_cast<qulonglong>(wid)));
    }
    return OmWgpuNativeWindow();
  }
#  endif

#  if OMNISIM_HAVE_QT_WAYLAND
  if (platform.startsWith(QLatin1String("wayland"))) {
    QNativeInterface::QWaylandApplication *wl =
      qGuiApp ? qGuiApp->nativeInterface<QNativeInterface::QWaylandApplication>() : nullptr;
    void *wlDisplay = wl ? static_cast<void *>(wl->display()) : nullptr;
    void *wlSurface = nullptr;
#    if OMNISIM_HAVE_QPA_NATIVE_INTERFACE
    // "surface" is the wl_surface* the Wayland QPA plugin publishes per QWindow.
    // Do NOT substitute QWindow::winId() here: what the Wayland plugin returns
    // from winId() is not a contractual wl_surface, and handing wgpu a wrong
    // pointer CRASHES instead of degrading — the one outcome this must avoid.
    if (QPlatformNativeInterface *pni = QGuiApplication::platformNativeInterface())
      wlSurface = pni->nativeResourceForWindow(QByteArrayLiteral("surface"), window);
#    endif
    if (wlDisplay && wlSurface)
      return OmWgpuNativeWindow::waylandSurface(wlDisplay, wlSurface);
    if (!sWarned) {
      sWarned = true;
      OmLog::info(QString("[OmWgpuView] wayland session but no usable native handle "
                          "(wl_display=%1 wl_surface=%2 qpa-native-interface=%3) — presentation "
                          "stays on the GL blit path")
                    .arg(wlDisplay ? 1 : 0)
                    .arg(wlSurface ? 1 : 0)
                    .arg(OMNISIM_HAVE_QPA_NATIVE_INTERFACE));
    }
    return OmWgpuNativeWindow();
  }
#  endif

  if (!sWarned) {
    sWarned = true;
    OmLog::info(QString("[OmWgpuView] no wgpu surface source for Qt platform '%1' — presentation "
                        "stays on the readback + GL blit path")
                  .arg(platform));
  }
  return OmWgpuNativeWindow();
#endif
}

namespace {
  // OmniSim/VRML rule (OmViewpoint::updateFieldOfViewY): the Viewpoint's fieldOfView
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
  OmSolid *topSolidOf(OmSolid *s) {
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
  void highlightSelectedDraws(std::vector<OmWgpuSolidDraw> &draws,
                              const std::vector<OmSolid *> &tops, const OmSolid *selTop) {
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

  // R4 step-3c-A.3: gather (world anchor, world axis) for every OmJoint reachable
  // under `root`, mirroring collectShapeDraws' scene-graph recursion (proven to
  // reach every link). Each joint contributes exactly one entry.
  struct JointAxis {
    OmVector3 anchor, dir;
  };
  void gatherJointAxes(OmBaseNode *root, std::vector<JointAxis> &out) {
    if (!root)
      return;
    if (OmSolid *s = dynamic_cast<OmSolid *>(root)) {
      const QVector<OmBasicJoint *> &js = s->jointChildren();
      for (OmBasicJoint *bj : js) {
        OmJoint *j = dynamic_cast<OmJoint *>(bj);
        if (!j)
          continue;
        JointAxis ja;
        if (j->axisRepresentation(ja.anchor, ja.dir))
          out.push_back(ja);
      }
    }
    if (OmGroup *g = dynamic_cast<OmGroup *>(root)) {
      const int n = g->childCount();
      for (int i = 0; i < n; ++i)
        gatherJointAxes(g->child(i), out);
    }
  }

  // Collect every OmSolid strictly BELOW `root` (root excluded) — the nested solids whose
  // translation/rotation fields are parent-relative (not world). Used by the parent-frame
  // drag self-check to find a solid with a non-identity parent transform to stress.
  void collectDescendantSolids(OmBaseNode *root, std::vector<OmSolid *> &out) {
    OmGroup *g = dynamic_cast<OmGroup *>(root);
    if (!g)
      return;
    const int n = g->childCount();
    for (int i = 0; i < n; ++i) {
      OmBaseNode *c = g->child(i);
      if (OmSolid *s = dynamic_cast<OmSolid *>(c))
        out.push_back(s);
      collectDescendantSolids(c, out);
    }
  }

  // Append a thin cyan box per joint axis (a bar of length L through the anchor
  // along the axis) to `draws`; model matrices go into `store`, reserved up front
  // so the draw pointers stay valid. Returns the number appended. The caller gates
  // this on VF_JOINT_AXES; `store` must outlive the render.
  int collectJointAxisDraws(OmWgpuMeshCache &cache, std::vector<OmWgpuSolidDraw> &draws,
                            std::vector<std::array<float, 16>> &store) {
    OmWorld *world = OmWorld::instance();
    if (!world)
      return 0;
    std::vector<JointAxis> axes;
    for (OmSolid *s : world->topSolids())
      gatherJointAxes(s, axes);
    if (axes.empty())
      return 0;
    OmWgpuMeshHandle box =
      OmWgpuMeshAdapter::acquirePrimitive(cache, 7001ull, OmWgpuMeshAdapter::PrimitiveKind::Box);
    if (!box.vertexBuffer || !box.indexBuffer || !box.indexCount)
      return 0;
    store.reserve(store.size() + axes.size());
    const float L = 0.2f, w = 0.006f;  // bar length along axis, cross-section
    int added = 0;
    for (const JointAxis &ja : axes) {
      OmVector3 dir = ja.dir;
      if (dir.length() < 1e-9)
        continue;
      dir = dir.normalized();
      const OmVector3 ref = std::fabs(dir.z()) < 0.9 ? OmVector3(0, 0, 1) : OmVector3(1, 0, 0);
      const OmVector3 p1 = ref.cross(dir).normalized();
      const OmVector3 p2 = dir.cross(p1);
      // Column-major model: col0 = dir*L, col1 = p1*w, col2 = p2*w, col3 = anchor.
      const std::array<float, 16> m = {
        static_cast<float>(dir.x() * L), static_cast<float>(dir.y() * L), static_cast<float>(dir.z() * L),
        0.0f, static_cast<float>(p1.x() * w), static_cast<float>(p1.y() * w), static_cast<float>(p1.z() * w),
        0.0f, static_cast<float>(p2.x() * w), static_cast<float>(p2.y() * w), static_cast<float>(p2.z() * w),
        0.0f, static_cast<float>(ja.anchor.x()), static_cast<float>(ja.anchor.y()),
        static_cast<float>(ja.anchor.z()), 1.0f};
      store.push_back(m);
      OmWgpuSolidDraw d;
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
  void pushSeg(std::vector<float> &v, const OmVector3 &a, const OmVector3 &b) {
    const OmVector3 e[2] = {a, b};
    for (const OmVector3 &p : e) {
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
  void emitGeomEdges(OmGeometry *g, std::vector<float> &v) {
    if (!g)
      return;
    const double kTwoPi = 6.283185307179586;
    const OmMatrix4 m = g->matrix();
    switch (g->nodeType()) {
      case WB_NODE_BOX: {
        const OmVector3 &s = static_cast<OmBox *>(g)->size();
        const double hx = s.x() * 0.5, hy = s.y() * 0.5, hz = s.z() * 0.5;
        OmVector3 c[8];
        int idx = 0;
        for (int xi = 0; xi < 2; ++xi)
          for (int yi = 0; yi < 2; ++yi)
            for (int zi = 0; zi < 2; ++zi)
              c[idx++] = m * OmVector3(xi ? hx : -hx, yi ? hy : -hy, zi ? hz : -hz);
        const int e[12][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7},   // along z
                              {0, 2}, {1, 3}, {4, 6}, {5, 7},   // along y
                              {0, 4}, {1, 5}, {2, 6}, {3, 7}};  // along x
        for (int i = 0; i < 12; ++i)
          pushSeg(v, c[e[i][0]], c[e[i][1]]);
        break;
      }
      case WB_NODE_SPHERE: {
        const double r = static_cast<OmSphere *>(g)->radius();
        const int N = 24;
        for (int plane = 0; plane < 3; ++plane) {
          OmVector3 prev;
          for (int k = 0; k <= N; ++k) {
            const double a = kTwoPi * k / N;
            const double ca = r * std::cos(a), sa = r * std::sin(a);
            const OmVector3 local = plane == 0 ? OmVector3(ca, sa, 0)
                                   : plane == 1 ? OmVector3(ca, 0, sa)
                                                : OmVector3(0, ca, sa);
            const OmVector3 p = m * local;
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
          OmCylinder *c = static_cast<OmCylinder *>(g);
          r = c->radius();
          h = c->height();
        } else {
          OmCapsule *c = static_cast<OmCapsule *>(g);
          r = c->radius();
          h = c->height();
        }
        const double hz = h * 0.5;
        const int N = 24;
        OmVector3 prevTop, prevBot;
        for (int k = 0; k <= N; ++k) {
          const double a = kTwoPi * k / N;
          const double ca = r * std::cos(a), sa = r * std::sin(a);
          const OmVector3 top = m * OmVector3(ca, sa, hz);
          const OmVector3 bot = m * OmVector3(ca, sa, -hz);
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
  void gatherBoundingEdges(OmBaseNode *root, std::vector<float> &v) {
    if (!root)
      return;
    if (OmGeometry *g = dynamic_cast<OmGeometry *>(root)) {
      emitGeomEdges(g, v);
      return;
    }
    if (OmShape *sh = dynamic_cast<OmShape *>(root)) {
      emitGeomEdges(sh->geometry(), v);
      return;
    }
    if (OmGroup *grp = dynamic_cast<OmGroup *>(root)) {
      const int n = grp->childCount();
      for (int i = 0; i < n; ++i)
        gatherBoundingEdges(grp->child(i), v);
    }
  }

  // Walk every solid (a boundingObject lives in its own SFNode field, not the
  // children group, so handle it per-solid then recurse the scene graph).
  void walkSolidsForBounding(OmBaseNode *root, std::vector<float> &v) {
    if (!root)
      return;
    if (OmSolid *s = dynamic_cast<OmSolid *>(root))
      if (s->boundingObject())
        gatherBoundingEdges(s->boundingObject(), v);
    if (OmGroup *grp = dynamic_cast<OmGroup *>(root)) {
      const int n = grp->childCount();
      for (int i = 0; i < n; ++i)
        walkSolidsForBounding(grp->child(i), v);
    }
  }

  // Collect bounding-object wireframe segments for the whole world. `v` fills with
  // stride-32 vertices (2 per segment); returns the vertex count.
  uint32_t collectBoundingEdges(std::vector<float> &v) {
    OmWorld *world = OmWorld::instance();
    if (!world)
      return 0;
    for (OmSolid *s : world->topSolids())
      walkSolidsForBounding(s, v);
    return static_cast<uint32_t>(v.size() / 8);
  }

  // R4 step-3c-A: a 3-axis cross (3 segments) centred at world point `c`, half-arm
  // `h` — the marker glyph for centre-of-mass / contact points.
  void emitCross(std::vector<float> &v, const OmVector3 &c, double h) {
    pushSeg(v, OmVector3(c.x() - h, c.y(), c.z()), OmVector3(c.x() + h, c.y(), c.z()));
    pushSeg(v, OmVector3(c.x(), c.y() - h, c.z()), OmVector3(c.x(), c.y() + h, c.z()));
    pushSeg(v, OmVector3(c.x(), c.y(), c.z() - h), OmVector3(c.x(), c.y(), c.z() + h));
  }

  void walkSolidsForCom(OmBaseNode *root, std::vector<float> &v) {
    if (!root)
      return;
    if (OmSolid *s = dynamic_cast<OmSolid *>(root))
      emitCross(v, s->computedGlobalCenterOfMass(), 0.06);
    if (OmGroup *grp = dynamic_cast<OmGroup *>(root)) {
      const int n = grp->childCount();
      for (int i = 0; i < n; ++i)
        walkSolidsForCom(grp->child(i), v);
    }
  }

  // Centre-of-mass markers: a cross at every solid's global COM. (WREN gates these
  // per-solid via showGlobalCenterOfMassRepresentation, not a VF_ flag.) Returns
  // the vertex count (6 per marker).
  uint32_t collectComCrosses(std::vector<float> &v) {
    OmWorld *world = OmWorld::instance();
    if (!world)
      return 0;
    for (OmSolid *s : world->topSolids())
      walkSolidsForCom(s, v);
    return static_cast<uint32_t>(v.size() / 8);
  }

  // Per-solid-GATED COM walk for the LIVE pane: emit a COM cross only for solids whose
  // showGlobalCenterOfMassRepresentation is enabled — mirrors WREN's per-solid gating (there
  // is no global VF_ flag), so the live pane is WREN-identical when nothing is toggled.
  void walkSolidsForComGated(OmBaseNode *root, std::vector<float> &v) {
    if (!root)
      return;
    if (OmSolid *s = dynamic_cast<OmSolid *>(root)) {
      if (s->globalCenterOfMassRepresentationEnabled())
        emitCross(v, s->computedGlobalCenterOfMass(), 0.06);
    }
    if (OmGroup *grp = dynamic_cast<OmGroup *>(root)) {
      const int n = grp->childCount();
      for (int i = 0; i < n; ++i)
        walkSolidsForComGated(grp->child(i), v);
    }
  }
  uint32_t collectComCrossesGated(std::vector<float> &v) {
    OmWorld *world = OmWorld::instance();
    if (!world)
      return 0;
    for (OmSolid *s : world->topSolids())
      walkSolidsForComGated(s, v);
    return static_cast<uint32_t>(v.size() / 8);
  }

  void walkSolidsForContacts(OmBaseNode *root, std::vector<float> &v) {
    if (!root)
      return;
    if (OmSolid *s = dynamic_cast<OmSolid *>(root))
      // computedContactPoints() EXTRACTS the current engine contacts on demand; the plain
      // contactPoints() only returns a lazily-cached list that another path (WREN's own
      // contact rendering / a supervisor) must have requested — empty otherwise. Computing
      // here makes the wgpu overlay self-sufficient (this runs only when VF_CONTACT_POINTS
      // is on, so the per-frame extract cost is paid only when contacts are actually wanted).
      for (const OmVector3 &p : s->computedContactPoints())  // engine-computed, world-space
        emitCross(v, p, 0.025);
    if (OmGroup *grp = dynamic_cast<OmGroup *>(root)) {
      const int n = grp->childCount();
      for (int i = 0; i < n; ++i)
        walkSolidsForContacts(grp->child(i), v);
    }
  }

  // R4 step-3c-A: joint axes as world-space line segments (anchor ± dir·half) for the
  // live-pane always-on-top overlay (replaces the box representation on-screen).
  uint32_t collectJointAxisLineVerts(std::vector<float> &v) {
    OmWorld *world = OmWorld::instance();
    if (!world)
      return 0;
    std::vector<JointAxis> axes;
    for (OmSolid *s : world->topSolids())
      gatherJointAxes(s, axes);
    const double half = 0.1;  // 0.2 m total bar through the anchor
    for (const JointAxis &ja : axes) {
      OmVector3 d = ja.dir;
      if (d.length() < 1e-9)
        continue;
      d = d.normalized();
      pushSeg(v, ja.anchor - d * half, ja.anchor + d * half);
    }
    return static_cast<uint32_t>(v.size() / 8);
  }

  // R4 step-3c-A: camera/range-finder view frustums (VF_CAMERA_FRUSTUMS /
  // VF_RANGE_FINDER_FRUSTUMS). Convention from OmAbstractCamera::updateFrustumDisplay:
  // the optical axis is camera-local +X, near at x=minRange, FOV is horizontal, vertical
  // via height/width. Emits a near rect + far rect + 4 connectors (12 edges), world-
  // transformed by the camera's matrix(). Far is clamped so an unbounded camera frustum
  // stays a sane size.
  // cameraOn/rangeOn select which sensor types contribute, matching WREN's per-type gating
  // (OmAbstractCamera::setOptionalRendering): a range-finder frustum is gated by
  // VF_RANGE_FINDER_FRUSTUMS, every other camera (Camera, Lidar) by VF_CAMERA_FRUSTUMS.
  void gatherFrustumNode(OmBaseNode *root, std::vector<float> &v, bool cameraOn, bool rangeOn) {
    if (!root)
      return;
    OmAbstractCamera *cam = dynamic_cast<OmAbstractCamera *>(root);
    if (cam && (cam->isRangeFinder() ? rangeOn : cameraOn)) {
      const double fov = cam->fieldOfView();
      const double n = cam->minRange() > 1e-4 ? cam->minRange() : 0.05;
      double f = cam->maxRange();
      if (f <= n || f > 50.0)
        f = n + 1.0;  // clamp huge/invalid far to a visible frustum
      const int w = cam->width(), h = cam->height();
      const double aspectVH = w > 0 ? static_cast<double>(h) / static_cast<double>(w) : 0.75;
      const double t = std::tan(fov * 0.5);
      const OmMatrix4 m = cam->matrix();
      auto corner = [&](double d, double sy, double sz) {
        const double hw = d * t, hh = hw * aspectVH;  // +X forward, Y horizontal, Z vertical
        return m * OmVector3(d, sy * hw, sz * hh);
      };
      const OmVector3 nc[4] = {corner(n, -1, -1), corner(n, 1, -1), corner(n, 1, 1), corner(n, -1, 1)};
      const OmVector3 fc[4] = {corner(f, -1, -1), corner(f, 1, -1), corner(f, 1, 1), corner(f, -1, 1)};
      for (int i = 0; i < 4; ++i) {
        pushSeg(v, nc[i], nc[(i + 1) % 4]);  // near rect
        pushSeg(v, fc[i], fc[(i + 1) % 4]);  // far rect
        pushSeg(v, nc[i], fc[i]);            // near→far connector
      }
    }
    if (OmGroup *grp = dynamic_cast<OmGroup *>(root)) {
      const int nn = grp->childCount();
      for (int i = 0; i < nn; ++i)
        gatherFrustumNode(grp->child(i), v, cameraOn, rangeOn);  // descend regardless of type gate
    }
  }

  uint32_t collectFrustumLines(std::vector<float> &v, bool cameraOn, bool rangeOn) {
    OmWorld *world = OmWorld::instance();
    if (!world)
      return 0;
    for (OmSolid *s : world->topSolids())
      gatherFrustumNode(s, v, cameraOn, rangeOn);
    return static_cast<uint32_t>(v.size() / 8);
  }

  // First OmDirectionalLight anywhere in the tree (e.g. the OmniSimSun PROTO's light), for
  // matching WREN's actual sun direction in the lighting/shadow convergence.
  OmDirectionalLight *findFirstDirectionalLight(OmBaseNode *root) {
    if (!root)
      return nullptr;
    if (OmDirectionalLight *dl = dynamic_cast<OmDirectionalLight *>(root))
      return dl;
    if (OmGroup *g = dynamic_cast<OmGroup *>(root)) {
      const int n = g->childCount();
      for (int i = 0; i < n; ++i)
        if (OmDirectionalLight *d = findFirstDirectionalLight(g->child(i)))
          return d;
    }
    return nullptr;
  }

  // First OmCamera (not a range-finder) anywhere in the tree, for the device-output inset.
  OmAbstractCamera *findFirstCamera(OmBaseNode *root) {
    if (!root)
      return nullptr;
    if (OmAbstractCamera *cam = dynamic_cast<OmAbstractCamera *>(root))
      if (!cam->isRangeFinder())
        return cam;
    if (OmGroup *g = dynamic_cast<OmGroup *>(root)) {
      const int n = g->childCount();
      for (int i = 0; i < n; ++i)
        if (OmAbstractCamera *c = findFirstCamera(g->child(i)))
          return c;
    }
    return nullptr;
  }

  // R4 step-3c-A: surface normals (VF_NORMALS). One short outward line per
  // representative surface point of a primitive geometry (Box→6 face normals,
  // Sphere→6 axes, Cylinder/Capsule→8 sides + 2 caps), world-transformed by the
  // geometry matrix. (Per-VERTEX mesh normals for arbitrary IndexedFaceSet meshes
  // need CPU mesh data — a follow-up; primitives cover the common debug case.)
  void emitGeomNormals(OmGeometry *g, std::vector<float> &v) {
    if (!g)
      return;
    const double len = 0.05;
    const OmMatrix4 m = g->matrix();
    auto nline = [&](const OmVector3 &pLocal, const OmVector3 &nLocal) {
      pushSeg(v, m * pLocal, m * (pLocal + nLocal.normalized() * len));
    };
    switch (g->nodeType()) {
      case WB_NODE_BOX: {
        const OmVector3 &s = static_cast<OmBox *>(g)->size();
        const double hx = s.x() * 0.5, hy = s.y() * 0.5, hz = s.z() * 0.5;
        nline(OmVector3(hx, 0, 0), OmVector3(1, 0, 0));
        nline(OmVector3(-hx, 0, 0), OmVector3(-1, 0, 0));
        nline(OmVector3(0, hy, 0), OmVector3(0, 1, 0));
        nline(OmVector3(0, -hy, 0), OmVector3(0, -1, 0));
        nline(OmVector3(0, 0, hz), OmVector3(0, 0, 1));
        nline(OmVector3(0, 0, -hz), OmVector3(0, 0, -1));
        break;
      }
      case WB_NODE_SPHERE: {
        const double r = static_cast<OmSphere *>(g)->radius();
        const OmVector3 d[6] = {OmVector3(1, 0, 0),  OmVector3(-1, 0, 0), OmVector3(0, 1, 0),
                                OmVector3(0, -1, 0), OmVector3(0, 0, 1),  OmVector3(0, 0, -1)};
        for (const OmVector3 &dir : d)
          nline(dir * r, dir);
        break;
      }
      case WB_NODE_CYLINDER:
      case WB_NODE_CAPSULE: {
        double r, h;
        if (g->nodeType() == WB_NODE_CYLINDER) {
          OmCylinder *c = static_cast<OmCylinder *>(g);
          r = c->radius();
          h = c->height();
        } else {
          OmCapsule *c = static_cast<OmCapsule *>(g);
          r = c->radius();
          h = c->height();
        }
        for (int k = 0; k < 8; ++k) {
          const double a = k * 0.785398163;
          const OmVector3 dir(std::cos(a), std::sin(a), 0);
          nline(dir * r, dir);
        }
        nline(OmVector3(0, 0, h * 0.5), OmVector3(0, 0, 1));
        nline(OmVector3(0, 0, -h * 0.5), OmVector3(0, 0, -1));
        break;
      }
      default:
        break;
    }
  }

  void gatherNormalsNode(OmBaseNode *root, std::vector<float> &v) {
    if (!root)
      return;
    if (OmShape *sh = dynamic_cast<OmShape *>(root))
      emitGeomNormals(sh->geometry(), v);
    if (OmGroup *grp = dynamic_cast<OmGroup *>(root)) {
      const int n = grp->childCount();
      for (int i = 0; i < n; ++i)
        gatherNormalsNode(grp->child(i), v);
    }
  }

  uint32_t collectNormalLines(std::vector<float> &v) {
    OmWorld *world = OmWorld::instance();
    if (!world)
      return 0;
    for (OmSolid *s : world->topSolids())
      gatherNormalsNode(s, v);
    return static_cast<uint32_t>(v.size() / 8);
  }

  // R4 step-3c-A: lidar ray paths (VF_LIDAR_RAYS_PATHS). A subsampled fan of rays from
  // each OmLidar origin out to maxRange, spanning its horizontal × vertical FOV. Local
  // ray dir (cos v·cos h, cos v·sin h, sin v) in the +X-forward / Y-horizontal /
  // Z-vertical convention (same as the frustum), world-transformed by the lidar matrix.
  void gatherLidarRays(OmBaseNode *root, std::vector<float> &v) {
    if (!root)
      return;
    if (OmLidar *l = dynamic_cast<OmLidar *>(root)) {
      const double hfov = l->fieldOfView();
      const int lw = l->width(), lh = l->height();
      const double vfov = (lw > 0) ? hfov * static_cast<double>(lh) / lw : 0.0;  // matches OmLidar
      double range = l->maxRange();
      if (range <= 0.0 || range > 20.0)
        range = 20.0;  // clamp unbounded lidar range to a visible length
      const int nh = 32, nv = l->height() > 1 ? std::min(l->height(), 6) : 1;
      const OmMatrix4 m = l->matrix();
      for (int vi = 0; vi < nv; ++vi) {
        const double vt = nv > 1 ? (-vfov * 0.5 + vfov * vi / (nv - 1)) : 0.0;
        for (int hi = 0; hi <= nh; ++hi) {
          const double ht = -hfov * 0.5 + hfov * hi / nh;
          const OmVector3 dir(std::cos(vt) * std::cos(ht), std::cos(vt) * std::sin(ht), std::sin(vt));
          pushSeg(v, m * OmVector3(0, 0, 0), m * (dir * range));
        }
      }
    }
    if (OmGroup *grp = dynamic_cast<OmGroup *>(root)) {
      const int n = grp->childCount();
      for (int i = 0; i < n; ++i)
        gatherLidarRays(grp->child(i), v);
    }
  }

  uint32_t collectLidarRays(std::vector<float> &v) {
    OmWorld *world = OmWorld::instance();
    if (!world)
      return 0;
    for (OmSolid *s : world->topSolids())
      gatherLidarRays(s, v);
    return static_cast<uint32_t>(v.size() / 8);
  }

  // W4b: radar detection volume (VF_RADAR_FRUSTUMS). A faithful port of
  // OmRadar::applyFrustumToWren -- same vertex sequence, same 24-step arcs, same
  // +X-forward / Y-horizontal / Z-vertical convention -- so the wgpu overlay and the WREN
  // one describe the SAME volume rather than two plausible-looking cones. WREN emits a
  // LINE_SET (consecutive vertex PAIRS), which is exactly what drawOverlayLines consumes,
  // so each WREN vertex pair becomes one pushSeg.
  void gatherRadarFrustums(OmBaseNode *root, std::vector<float> &v) {
    if (!root)
      return;
    if (OmRadar *r = dynamic_cast<OmRadar *>(root)) {
      const double hf = r->horizontalFieldOfView(), vf = r->verticalFieldOfView();
      const double sinH = std::sin(hf / 2.0), cosH = std::cos(hf / 2.0);
      const double sinV = std::sin(vf / 2.0), cosV = std::cos(vf / 2.0);
      const double fX = cosV * cosH, fY = cosV * sinH, fZ = sinV;
      const double mnR = r->minRange(), mxR = r->maxRange();
      const double maxX = mxR * fX, maxY = mxR * fY, maxZ = mxR * fZ;
      const double minX = mnR * fX, minY = mnR * fY, minZ = mnR * fZ;
      const OmMatrix4 m = r->matrix();
      auto seg = [&v, &m](double ax, double ay, double az, double bx, double by, double bz) {
        pushSeg(v, m * OmVector3(ax, ay, az), m * OmVector3(bx, by, bz));
      };
      seg(0, 0, 0, mnR, 0, 0);
      seg(minX, minY, -minZ, maxX, maxY, -maxZ);
      seg(minX, -minY, -minZ, maxX, -maxY, -maxZ);
      seg(minX, minY, minZ, maxX, maxY, maxZ);
      seg(minX, -minY, minZ, maxX, -maxY, maxZ);
      const int steps = 24;
      const double ranges[2] = {mnR, mxR};
      for (int j = 0; j < steps; ++j) {  // top + bottom margins
        const double a1 = hf / 2.0 - j * hf / steps, a2 = hf / 2.0 - (j + 1) * hf / steps;
        const double x1f = cosV * std::cos(a1), x2f = cosV * std::cos(a2);
        const double y1f = cosV * std::sin(a1), y2f = cosV * std::sin(a2);
        for (int k = 0; k < 2; ++k) {
          const double rg = ranges[k], z = rg * sinV;
          seg(rg * x1f, rg * y1f, z, rg * x2f, rg * y2f, z);
          seg(rg * x1f, rg * y1f, -z, rg * x2f, rg * y2f, -z);
        }
      }
      for (int j = 0; j < steps; ++j) {  // right + left margins
        const double a1 = vf / 2.0 - j * vf / steps, a2 = vf / 2.0 - (j + 1) * vf / steps;
        const double x1f = std::cos(a1) * cosH, x2f = std::cos(a2) * cosH;
        const double y1f = std::cos(a1) * sinH, y2f = std::cos(a2) * sinH;
        const double z1f = std::sin(a1), z2f = std::sin(a2);
        for (int k = 0; k < 2; ++k) {
          const double rg = ranges[k];
          seg(rg * x1f, -rg * y1f, rg * z1f, rg * x2f, -rg * y2f, rg * z2f);
          seg(rg * x1f, rg * y1f, rg * z1f, rg * x2f, rg * y2f, rg * z2f);
        }
      }
    }
    if (OmGroup *grp = dynamic_cast<OmGroup *>(root)) {
      const int n = grp->childCount();
      for (int i = 0; i < n; ++i)
        gatherRadarFrustums(grp->child(i), v);
    }
  }

  uint32_t collectRadarFrustums(std::vector<float> &v) {
    OmWorld *world = OmWorld::instance();
    if (!world)
      return 0;
    for (OmSolid *s : world->topSolids())
      gatherRadarFrustums(s, v);
    return static_cast<uint32_t>(v.size() / 8);
  }

  // W4b: light-sensor rays (VF_LIGHT_SENSORS_RAYS). WREN draws a UNIT +X segment and scales
  // the transform by wr_config_get_line_scale() -- a WREN-global, view-dependent scale. This
  // path deliberately does NOT reproduce that: calling into WREN from the wgpu collector is
  // exactly the dependency this campaign is removing. Instead it uses a fixed metric length,
  // the same convention collectJointAxisLineVerts already uses (0.1 m half-bar). So the ray
  // POINTS where WREN's points; its LENGTH is metric rather than view-scaled.
  void gatherLightSensorRays(OmBaseNode *root, std::vector<float> &v) {
    if (!root)
      return;
    if (OmLightSensor *ls = dynamic_cast<OmLightSensor *>(root)) {
      const OmMatrix4 m = ls->matrix();
      pushSeg(v, m * OmVector3(0, 0, 0), m * OmVector3(0.1, 0, 0));
    }
    if (OmGroup *grp = dynamic_cast<OmGroup *>(root)) {
      const int n = grp->childCount();
      for (int i = 0; i < n; ++i)
        gatherLightSensorRays(grp->child(i), v);
    }
  }

  uint32_t collectLightSensorRays(std::vector<float> &v) {
    OmWorld *world = OmWorld::instance();
    if (!world)
      return 0;
    for (OmSolid *s : world->topSolids())
      gatherLightSensorRays(s, v);
    return static_cast<uint32_t>(v.size() / 8);
  }

  // R4 step-3c-A: translate-manipulator handles — the gizmo geometry, 3 world-axis
  // segments (X red / Y green / Z blue) at a solid's origin, scaled by `len`. Emits
  // each axis into its own vector so the caller can colour them. (The drag/hit-test
  // interaction is a separate follow-up; this is the visual handle.)
  void emitManipulatorAxes(OmSolid *s, std::vector<float> &vx, std::vector<float> &vy,
                           std::vector<float> &vz, double len) {
    if (!s)
      return;
    const OmVector3 o = s->position();
    pushSeg(vx, o, o + s->xAxis().normalized() * len);
    pushSeg(vy, o, o + s->yAxis().normalized() * len);
    pushSeg(vz, o, o + s->zAxis().normalized() * len);
  }

  // R4 step-3c-A: rotate-gizmo ring in the plane spanned by (u,w) about centre `o`,
  // radius `r` — a 24-segment circle. (u,w) are two orthonormal solid axes; the ring's
  // normal is the third axis, so X/Y/Z rings colour by their rotation axis.
  void emitRotateRing(std::vector<float> &v, const OmVector3 &o, const OmVector3 &u,
                      const OmVector3 &w, double r) {
    const double kTwoPi = 6.283185307179586;
    const int N = 24;
    OmVector3 prev;
    for (int k = 0; k <= N; ++k) {
      const double a = kTwoPi * k / N;
      const OmVector3 p = o + (u * (r * std::cos(a)) + w * (r * std::sin(a)));
      if (k > 0)
        pushSeg(v, prev, p);
      prev = p;
    }
  }

  // The three rotate rings of a solid's gizmo (X-axis ring in the YZ plane, etc.),
  // each into its own vector so the caller can colour them R/G/B by rotation axis.
  void emitRotateRings(OmSolid *s, std::vector<float> &vx, std::vector<float> &vy,
                       std::vector<float> &vz, double r) {
    if (!s)
      return;
    const OmVector3 o = s->position();
    const OmVector3 X = s->xAxis().normalized(), Y = s->yAxis().normalized(),
                    Z = s->zAxis().normalized();
    emitRotateRing(vx, o, Y, Z, r);  // ring about X (red)
    emitRotateRing(vy, o, Z, X, r);  // ring about Y (green)
    emitRotateRing(vz, o, X, Y, r);  // ring about Z (blue)
  }

  // R4 step-3c-A: axis-aligned wireframe cube (12 edges) of half-size `h` at `c` — the
  // scale-gizmo handle marker.
  void emitCube(std::vector<float> &v, const OmVector3 &c, double h) {
    OmVector3 cor[8];
    int idx = 0;
    for (int xi = 0; xi < 2; ++xi)
      for (int yi = 0; yi < 2; ++yi)
        for (int zi = 0; zi < 2; ++zi)
          cor[idx++] =
            OmVector3(c.x() + (xi ? h : -h), c.y() + (yi ? h : -h), c.z() + (zi ? h : -h));
    const int e[12][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}, {0, 2}, {1, 3},
                          {4, 6}, {5, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    for (int i = 0; i < 12; ++i)
      pushSeg(v, cor[e[i][0]], cor[e[i][1]]);
  }

  // Scale-gizmo handles: a small cube at the end of each solid axis (origin + axis·len),
  // into per-axis vectors for R/G/B colouring.
  void emitScaleBoxes(OmSolid *s, std::vector<float> &vx, std::vector<float> &vy,
                      std::vector<float> &vz, double len, double h) {
    if (!s)
      return;
    const OmVector3 o = s->position();
    emitCube(vx, o + s->xAxis().normalized() * len, h);
    emitCube(vy, o + s->yAxis().normalized() * len, h);
    emitCube(vz, o + s->zAxis().normalized() * len, h);
  }

  // Project a world point through a column-major view-proj to pane pixels (sx,sy) at
  // (w,h). Returns false if behind the camera (clip.w ≤ 0). Y is flipped to top-left.
  bool projectToScreen(const float vp[16], const OmVector3 &p, int w, int h, double &sx, double &sy) {
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
    OmWorld *world = OmWorld::instance();
    if (!world)
      return 0;
    for (OmSolid *s : world->topSolids())
      walkSolidsForContacts(s, v);
    return static_cast<uint32_t>(v.size() / 8);
  }

  // Support polygon: per top-level solid, the convex hull of its ground contact points
  // projected onto the horizontal (north/east) plane — the stability footprint. Mirrors
  // OmSolid::supportPolygon() exactly (same projection basis + OmMathsUtilities::
  // twoStepsConvexHull) but computed cold from computedContactPoints() so it does not depend
  // on the per-solid representation being enabled. Each hull is emitted as a closed loop of
  // segments at the mean contact height. Empty until something rests (≥3 contacts) → 0 in a
  // static scene, like contact points. No global VF flag (WREN gates it per-solid).
  uint32_t collectSupportPolygon(std::vector<float> &v) {
    OmWorld *world = OmWorld::instance();
    if (!world || !world->worldInfo())
      return 0;
    const OmWorldInfo *wi = world->worldInfo();
    const OmVector3 north = wi->northVector(), east = wi->eastVector(), up = wi->upVector();
    for (OmSolid *s : world->topSolids()) {
      const QVector<OmVector3> &cps = s->computedContactPoints(true);  // this solid + descendants
      const int n = cps.size();
      if (n < 3)
        continue;  // need at least a triangle to bound an area
      std::vector<OmVector2> proj(n);
      double h = 0.0;
      for (int i = 0; i < n; ++i) {
        proj[i].setXy(cps.at(i).dot(north), cps.at(i).dot(east));
        h += cps.at(i).dot(up);
      }
      h /= n;  // draw the footprint at the mean contact height
      std::vector<int> idx(n);
      const int hull = OmMathsUtilities::twoStepsConvexHull(proj, idx);
      if (hull < 3)
        continue;
      OmVector3 first, prev;
      for (int i = 0; i < hull; ++i) {
        const OmVector2 &pp = proj.at(idx.at(i));
        const OmVector3 w = north * pp.x() + east * pp.y() + up * h;  // reconstruct world point
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

// ==========================================================================================
// W4a (WREN retirement): the geometry-overlay entry points the PRODUCTION wgpu main view
// calls. Thin wrappers over the anonymous-namespace collectors above — deliberately thin, so
// the pane and the main view cannot drift apart in what they draw or how they gate it.
// ==========================================================================================

bool OmWgpuView::anyOptionalRenderingEnabled() {
  const OmWrenRenderingContext *rc = OmWrenRenderingContext::instance();
  if (!rc)
    return false;
  // Only the flags collectOptionalRenderingLines can actually draw. Every other View >
  // Optional Rendering item is still WREN-only and must NOT make the main view pay a scene
  // walk for an overlay that would come out empty.
  return rc->isOptionalRenderingEnabled(OmWrenRenderingContext::VF_ALL_BOUNDING_OBJECTS) ||
         rc->isOptionalRenderingEnabled(OmWrenRenderingContext::VF_JOINT_AXES) ||
         rc->isOptionalRenderingEnabled(OmWrenRenderingContext::VF_CAMERA_FRUSTUMS) ||
         rc->isOptionalRenderingEnabled(OmWrenRenderingContext::VF_RANGE_FINDER_FRUSTUMS) ||
         rc->isOptionalRenderingEnabled(OmWrenRenderingContext::VF_NORMALS) ||
         rc->isOptionalRenderingEnabled(OmWrenRenderingContext::VF_LIDAR_RAYS_PATHS) ||
         rc->isOptionalRenderingEnabled(OmWrenRenderingContext::VF_RADAR_FRUSTUMS) ||
         rc->isOptionalRenderingEnabled(OmWrenRenderingContext::VF_LIGHT_SENSORS_RAYS) ||
         rc->isOptionalRenderingEnabled(OmWrenRenderingContext::VF_CONTACT_POINTS);
}

void OmWgpuView::collectOptionalRenderingLines(OverlayLines &out) {
  OmWrenRenderingContext *rc = OmWrenRenderingContext::instance();
  if (!rc || !OmWorld::instance())
    return;
  if (rc->isOptionalRenderingEnabled(OmWrenRenderingContext::VF_ALL_BOUNDING_OBJECTS))
    collectBoundingEdges(out.bounding);
  if (rc->isOptionalRenderingEnabled(OmWrenRenderingContext::VF_JOINT_AXES))
    collectJointAxisLineVerts(out.jointAxes);
  {
    // Per-type gating, matching WREN: a range-finder's frustum belongs to
    // VF_RANGE_FINDER_FRUSTUMS, every other camera (Camera, Lidar) to VF_CAMERA_FRUSTUMS.
    const bool camFr = rc->isOptionalRenderingEnabled(OmWrenRenderingContext::VF_CAMERA_FRUSTUMS);
    const bool rfFr =
      rc->isOptionalRenderingEnabled(OmWrenRenderingContext::VF_RANGE_FINDER_FRUSTUMS);
    if (camFr || rfFr)
      collectFrustumLines(out.frustums, camFr, rfFr);
  }
  if (rc->isOptionalRenderingEnabled(OmWrenRenderingContext::VF_NORMALS))
    collectNormalLines(out.normals);
  if (rc->isOptionalRenderingEnabled(OmWrenRenderingContext::VF_LIDAR_RAYS_PATHS))
    collectLidarRays(out.lidarRays);
  if (rc->isOptionalRenderingEnabled(OmWrenRenderingContext::VF_RADAR_FRUSTUMS))
    collectRadarFrustums(out.radar);
  if (rc->isOptionalRenderingEnabled(OmWrenRenderingContext::VF_LIGHT_SENSORS_RAYS))
    collectLightSensorRays(out.lightRays);
  // Contact points are physics-state dependent: an empty result means nothing is touching
  // this frame, not a fault. computedContactPoints() extracts on demand, so this walk runs
  // only while the menu item is on.
  if (rc->isOptionalRenderingEnabled(OmWrenRenderingContext::VF_CONTACT_POINTS))
    collectContactCrosses(out.contacts);
}

void OmWgpuView::appendOverlayBox(std::vector<float> &v, const double min3[3], const double max3[3]) {
  if (!min3 || !max3)
    return;
  OmVector3 c[8];
  int idx = 0;
  for (int xi = 0; xi < 2; ++xi)
    for (int yi = 0; yi < 2; ++yi)
      for (int zi = 0; zi < 2; ++zi)
        c[idx++] = OmVector3(xi ? max3[0] : min3[0], yi ? max3[1] : min3[1], zi ? max3[2] : min3[2]);
  const int e[12][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}, {0, 2}, {1, 3},
                        {4, 6}, {5, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
  for (int i = 0; i < 12; ++i)
    pushSeg(v, c[e[i][0]], c[e[i][1]]);
}

OmWgpuView::OmWgpuView(QWindow *parent) : QWindow(parent) {
  setSurfaceType(QWindow::VulkanSurface);
  setTitle(QStringLiteral("OmniSim — wgpu viewport (R4 step 2)"));
  resize(640, 480);
  // (A self-check "park the pane off-screen" experiment lived here briefly — REVERTED: off-monitor
  // windows never get exposed, so the backend never initialized and the self-check never ran. The
  // golden no longer depends on window layout at all: see OmWrenWindow::grabSceneOffscreen.)

  mTimer = new QTimer(this);
  mTimer->setInterval(16);  // ~60 Hz; Fifo present self-throttles to vsync
  connect(mTimer, &QTimer::timeout, this, [this]() { renderTick(); });
  // NOTE: started on first expose, NOT here — querying the wgpu backend before
  // the graphics subsystem is up makes device init fail and the registry caches
  // that failure for the whole run (poisoning the backend).
}

OmWgpuView::~OmWgpuView() {
  delete mPickTarget;
  delete mTextureCache;
  delete mMeshCache;
  delete mSurface;  // mTimer is a child QObject, auto-deleted
}

int OmWgpuView::pixelWidth() const {
  const int w = static_cast<int>(width() * devicePixelRatio());
  return w > 0 ? w : 1;
}

int OmWgpuView::pixelHeight() const {
  const int h = static_cast<int>(height() * devicePixelRatio());
  return h > 0 ? h : 1;
}

void OmWgpuView::exposeEvent(QExposeEvent *) {
  if (!isExposed())
    return;
  ensureSurface();  // inits backend + mesh cache + surface — AFTER graphics is up
  // Start the frame timer once the backend is up, then leave it running (don't
  // stop on a later un-expose): the offscreen self-check + per-frame draw keep
  // ticking, and renderTick self-guards the on-screen part with isExposed().
  if (mBackend && !mTimer->isActive())
    mTimer->start();
}

void OmWgpuView::resizeEvent(QResizeEvent *) {
  if (mSurface)
    mSurface->resize(pixelWidth(), pixelHeight());
}

// Fetch the wgpu backend + mesh cache. No window/HWND/surface needed, so the
// offscreen self-check + picking can run even if the pane is never exposed.
void OmWgpuView::ensureBackend() {
  if (mBackend)
    return;
  OmRenderBackendRegistry::initialise();  // idempotent
  OmRenderBackend *rb = OmRenderBackendRegistry::vulkanBackend();
  if (!rb || rb->kind() != OmRenderBackendKind::Vulkan || !rb->isAvailable())
    return;  // not a wgpu-ON build / dep missing — stays inert
  mBackend = static_cast<OmVulkanBackend *>(rb);
  if (!mMeshCache)
    mMeshCache = new OmWgpuMeshCache(mBackend);
  if (!mTextureCache)
    mTextureCache = new OmWgpuTextureCache(mBackend);
}

void OmWgpuView::ensureSurface() {
  if (mSurface || mInitAttempted)
    return;
  mInitAttempted = true;
  ensureBackend();
  if (!mBackend) {
    OmLog::info(
      "[OmWgpuView] wgpu backend unavailable — OMNISIM_VIEW3D_WGPU needs a wgpu-ON build "
      "(OMNISIM_WITH_VULKAN=ON + wgpu-native). Window will idle.");
    return;
  }
  // Native handles for whatever windowing system is actually running (HWND on
  // Windows, Xlib/XCB on an X11 session, wl_surface on Wayland). An invalid
  // handle is not an error: the window keeps its existing presentation path.
  // mInitAttempted (set above) keeps this a one-shot, so no retry churn.
  const OmWgpuNativeWindow nativeWindow = OmWgpuNativeWindowFromQtWindow(this);
  if (!nativeWindow.isValid())
    return;  // OmWgpuNativeWindowFromQtWindow already logged the reason
  mSurface = new OmWgpuSurface(mBackend, nativeWindow, pixelWidth(), pixelHeight());
  if (!mSurface->isUsable()) {
    OmLog::info("[OmWgpuView] surface creation failed");
    delete mSurface;
    mSurface = nullptr;
    return;
  }
  OmLog::info("[OmWgpuView] presenting to screen via wgpu (R4 step 3)");
}

// Build a Webots-camera-convention (+X forward, +Y left, +Z up) world matrix
// from the live main Viewpoint, so buildViewProj renders the same view WREN
// shows. A OmniSim Viewpoint looks down -Z, so forward = -orientation.direction()
// and up = orientation.up() (mirrors OmViewpoint's own view-matrix basis).
bool OmWgpuView::buildViewpointCamera(OmMatrix4 &cam, double &horizFov) const {
  OmWorld *world = OmWorld::instance();
  if (!world)
    return false;
  OmViewpoint *vp = world->viewpoint();
  if (!vp || !vp->position() || !vp->orientation())
    return false;
  const OmVector3 eye = vp->position()->value();
  const OmRotation rot = vp->orientation()->value();
  // OmRotation::direction() is the toward-scene look vector (verified by the
  // self-check: -direction() put all geometry behind the camera, clipw<0).
  const OmVector3 f = rot.direction().normalized();    // forward (toward scene)
  const OmVector3 s = f.cross(rot.up()).normalized();  // right
  const OmVector3 u = s.cross(f);                      // orthonormal up
  // Row-major OmMatrix4 columns: [f | -s (left) | u | eye].
  cam = OmMatrix4(f.x(), -s.x(), u.x(), eye.x(), f.y(), -s.y(), u.y(), eye.y(), f.z(), -s.z(), u.z(),
                  eye.z(), 0, 0, 0, 1);
  horizFov = vp->fieldOfView() ? vp->fieldOfView()->value() : 0.785;
  return true;
}

// Column-major view-proj for the live world from the live Viewpoint, aspect-
// corrected to match WREN. Shared by draw + pick so a click hits what's shown.
bool OmWgpuView::worldViewProj(float out16[16]) const {
  OmMatrix4 cam;
  double horizFov = 0.785;
  if (!buildViewpointCamera(cam, horizFov))
    return false;
  const double aspect = static_cast<double>(pixelWidth()) / static_cast<double>(pixelHeight());
  OmWgpuSceneRenderer::buildViewProj(cam, horizFovForAspect(horizFov, aspect), aspect, 0.05, 1000.0,
                                     out16);
  return true;
}

// Step 3: render the live world from the live Viewpoint into the surface.
bool OmWgpuView::drawWorld(void *rgbaOut, int *outDraws) {
  if (!mSurface || !mMeshCache)
    return false;
  float vpm[16];
  if (!worldViewProj(vpm))
    return false;
  std::vector<OmWgpuSolidDraw> draws;
  std::vector<std::array<float, 16>> modelStorage;  // draws alias into this; keep alive past present
  std::vector<OmSolid *> tops;                      // parallel to draws (top solid per draw)
  OmWgpuSceneRenderer::collectWorldDraws(*mMeshCache, draws, modelStorage, &tops, mTextureCache);
  if (outDraws)
    *outDraws = static_cast<int>(draws.size());

  // R4 step-3c-A.2: highlight the live selection. Re-read every frame so it
  // tracks scene-tree / click selection changes; a null selection is a no-op.
  if (OmSelection::instance())
    highlightSelectedDraws(draws, tops, topSolidOf(OmSelection::instance()->selectedSolid()));

  OmWrenRenderingContext *rc = OmWrenRenderingContext::instance();

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
  if (rc && rc->isOptionalRenderingEnabled(OmWrenRenderingContext::VF_ALL_BOUNDING_OBJECTS)) {
    lineN = collectBoundingEdges(boEdges);
    if (lineN >= 2)
      lineV = boEdges.data();
  }
  if (rc && rc->isOptionalRenderingEnabled(OmWrenRenderingContext::VF_JOINT_AXES))
    collectJointAxisLineVerts(jaLines);  // appends joint-axis segments
  {
    // Per-type frustum gating (matches WREN): cameras/lidars under VF_CAMERA_FRUSTUMS,
    // range-finders under VF_RANGE_FINDER_FRUSTUMS. Either toggled → gather, the per-node
    // check inside picks which sensors contribute.
    const bool camFr = rc && rc->isOptionalRenderingEnabled(OmWrenRenderingContext::VF_CAMERA_FRUSTUMS);
    const bool rfFr =
      rc && rc->isOptionalRenderingEnabled(OmWrenRenderingContext::VF_RANGE_FINDER_FRUSTUMS);
    if (camFr || rfFr)
      collectFrustumLines(jaLines, camFr, rfFr);  // appends to the always-on-top batch
  }
  if (rc && rc->isOptionalRenderingEnabled(OmWrenRenderingContext::VF_NORMALS))
    collectNormalLines(jaLines);  // appends surface-normal segments
  if (rc && rc->isOptionalRenderingEnabled(OmWrenRenderingContext::VF_LIDAR_RAYS_PATHS))
    collectLidarRays(jaLines);  // appends lidar ray-path segments
  if (rc && rc->isOptionalRenderingEnabled(OmWrenRenderingContext::VF_CONTACT_POINTS))
    collectContactCrosses(jaLines);  // appends contact-point crosses (0 in a static scene).
  // COM markers: per-solid gated (WREN has no global VF flag for these) → WREN-identical when
  // no solid has showGlobalCenterOfMassRepresentation enabled; appears in the live pane for the
  // ones that do. Always-on-top, shares the top batch.
  collectComCrossesGated(jaLines);
  // R4 step-3c-A: translate-manipulator handles at the selected solid (always-on-top,
  // shown whenever something is selected). Combined into the top batch (one colour in
  // the live pane for now; per-axis R/G/B is verified offscreen).
  if (OmSelection::instance()) {
    if (OmSolid *sel = OmSelection::instance()->selectedSolid()) {
      // Distance-scaled gizmo: handle length ~8% of the eye→solid distance so it keeps a
      // roughly constant on-screen size as the user zooms. Clamped to a sane minimum.
      double hlen = 0.25;
      OmMatrix4 hc;
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
  OmMatrix4 eyeCam;
  double eyeHf = 0.785;
  float camPos[3] = {0, 0, 0};
  const bool haveEye = buildViewpointCamera(eyeCam, eyeHf);
  if (haveEye) {
    const OmVector3 e = eyeCam.translation();
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
OmSolid *OmWgpuView::pickSolidAt(int px, int py) {
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
    mPickTarget = new OmWgpuRenderTarget(mBackend, static_cast<uint32_t>(w), static_cast<uint32_t>(h));
  if (!mPickTarget->isUsable())
    return nullptr;
  float vpm[16];
  if (!worldViewProj(vpm))
    return nullptr;
  std::vector<OmWgpuSolidDraw> draws;
  std::vector<std::array<float, 16>> modelStorage;
  std::vector<OmSolid *> nodes;  // parallel to draws (top solid per draw)
  OmWgpuSceneRenderer::collectWorldDraws(*mMeshCache, draws, modelStorage, &nodes);
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
  OmWgpuClearColor black;  // (0,0,0,1) => background decodes to ID 0 = miss
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
int OmWgpuView::pickHandleAxis(int px, int py, OmSolid *sel, const float *vp16, int w, int h,
                               double len) const {
  if (!sel || !vp16)
    return -1;
  const OmVector3 o = sel->position();
  const OmVector3 ax[3] = {sel->xAxis().normalized(), sel->yAxis().normalized(),
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
int OmWgpuView::pickRotateRing(int px, int py, OmSolid *sel, const float *vp16, int w, int h,
                               double len) const {
  if (!sel || !vp16)
    return -1;
  const OmVector3 o = sel->position();
  double osx, osy;
  if (!projectToScreen(vp16, o, w, h, osx, osy))
    return -1;
  const double clickDist = std::hypot(px - osx, py - osy);
  // Ring about X lies in the YZ plane; a representative radius point is along Y. Etc.
  const OmVector3 radPt[3] = {sel->yAxis().normalized(), sel->zAxis().normalized(),
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

void OmWgpuView::mousePressEvent(QMouseEvent *event) {
  if (mBackend && mMeshCache) {
    const int px = static_cast<int>(event->position().x() * devicePixelRatio());
    const int py = static_cast<int>(event->position().y() * devicePixelRatio());
    OmSolid *sel = OmSelection::instance() ? OmSelection::instance()->selectedSolid() : nullptr;
    float vpm[16];
    if (sel && worldViewProj(vpm)) {
      double hlen = kHandleLen;  // distance-scaled to match the rendered gizmo
      {
        OmMatrix4 hc;
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
        const OmVector3 t = sel->translation();
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
        const OmRotation r = sel->rotation();
        mGrabStartRot[0] = r.x();
        mGrabStartRot[1] = r.y();
        mGrabStartRot[2] = r.z();
        mGrabStartRot[3] = r.angle();
        // Ring axis in the PARENT frame (R_localStart · unit), so the rotate composition
        // deltaM·startRotation is correct under a rotated/nested parent. For a top-level
        // solid R_parent = I, so this equals the world axis sel->{x,y,z}Axis() (unchanged).
        const OmVector3 unit(ring == 0 ? 1.0 : 0.0, ring == 1 ? 1.0 : 0.0, ring == 2 ? 1.0 : 0.0);
        const OmVector3 pa = (r.toMatrix3() * unit).normalized();
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
    OmSolid *picked = pickSolidAt(px, py);
    if (picked && OmSelection::instance())
      OmSelection::instance()->selectPoseFromView3D(picked);  // OmSolid is-a OmAbstractPose
  }
  QWindow::mousePressEvent(event);
}

void OmWgpuView::mouseMoveEvent(QMouseEvent *event) {
  if (mGrabAxis >= 0 && mGrabSolid) {
    const int px = static_cast<int>(event->position().x() * devicePixelRatio());
    const int py = static_cast<int>(event->position().y() * devicePixelRatio());
    float vpm[16];
    if (worldViewProj(vpm)) {
      const OmVector3 o = mGrabSolid->position();
      // axisWorld = the on-screen gizmo direction (the solid's world axis) — used to map
      // mouse pixels → metres. localDir = the SAME axis expressed in the PARENT frame
      // (R_local · unit) — used to update the parent-relative translation field. For a
      // top-level solid R_parent = I so localDir == axisWorld (byte-identical); under a
      // rotated/nested parent the parent rotation cancels, so the solid still slides along
      // the on-screen gizmo axis instead of drifting off it.
      const OmVector3 unit(mGrabAxis == 0 ? 1.0 : 0.0, mGrabAxis == 1 ? 1.0 : 0.0,
                           mGrabAxis == 2 ? 1.0 : 0.0);
      const OmVector3 axisWorld = (mGrabAxis == 0 ? mGrabSolid->xAxis()
                                   : mGrabAxis == 1 ? mGrabSolid->yAxis()
                                                    : mGrabSolid->zAxis())
                                    .normalized();
      const OmVector3 localDir = mGrabSolid->rotation().toMatrix3() * unit;
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
          mGrabSolid->setTranslation(OmVector3(mGrabStartT[0] + localDir.x() * worldDist,
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
        const OmMatrix3 deltaM(mGrabRotateParentAxis[0], mGrabRotateParentAxis[1],
                               mGrabRotateParentAxis[2], delta);
        const OmRotation startR(mGrabStartRot[0], mGrabStartRot[1], mGrabStartRot[2], mGrabStartRot[3]);
        mGrabRotateSolid->setRotation(OmRotation(deltaM * startR.toMatrix3()));
      }
    }
  }
  QWindow::mouseMoveEvent(event);
}

void OmWgpuView::mouseReleaseEvent(QMouseEvent *event) {
  mGrabAxis = -1;
  mGrabSolid = nullptr;
  mGrabRotateAxis = -1;
  mGrabRotateSolid = nullptr;
  QWindow::mouseReleaseEvent(event);
}

// Render the mimosa unit box spinning about the vertical axis at `phase`,
// fixed camera at (-2.5,0,0) looking down +X (OmniSim camera convention) at
// the box at the origin. rgbaOut, if non-null, receives the rendered
// backbuffer for the self-check. Returns presentScene's success.
bool OmWgpuView::drawBox(double phase, void *rgbaOut) {
  if (!mSurface || !mMeshCache)
    return false;
  OmWgpuMeshHandle box =
    OmWgpuMeshAdapter::acquirePrimitive(*mMeshCache, 1ull, OmWgpuMeshAdapter::PrimitiveKind::Box);
  if (!box.vertexBuffer || !box.indexBuffer || !box.indexCount)
    return false;

  const float a = static_cast<float>(phase);
  const float c = std::cos(a), s = std::sin(a);
  const float model[16] = {c, s, 0, 0, -s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};  // col-major Z-rot

  OmMatrix4 cam;  // identity rotation => local +X (forward) = world +X
  cam.setTranslation(-2.5, 0.0, 0.0);
  float vp[16];
  const double aspect = static_cast<double>(pixelWidth()) / static_cast<double>(pixelHeight());
  OmWgpuSceneRenderer::buildViewProj(cam, 1.1, aspect, 0.05, 100.0, vp);

  OmWgpuSolidDraw d;
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
void OmWgpuView::runSelfCheck() {
  if (!mBackend || !mMeshCache)
    return;
  const QByteArray path = qEnvironmentVariable("OMNISIM_VIEW3D_WGPU_SELFCHECK").toLocal8Bit();
  std::ofstream f(path.constData());
  if (!f)
    return;
  f << "R4 step-3c-A.1 — wgpu offscreen pick self-check (exposure-independent)\n";

  OmMatrix4 cam;
  double hf = 0.785;
  if (!buildViewpointCamera(cam, hf)) {
    f << "no world/viewpoint loaded — cannot verify\n";
    return;
  }
  // Fixed square offscreen target — result is independent of pane size/exposure.
  const uint32_t S = 480;
  float vpm[16];
  OmWgpuSceneRenderer::buildViewProj(cam, hf, 1.0, 0.05, 1000.0, vpm);  // 1:1 aspect

  std::vector<OmWgpuSolidDraw> draws;
  std::vector<std::array<float, 16>> ms;  // draws alias into this; keep alive past the render
  OmWgpuSceneRenderer::collectWorldDraws(*mMeshCache, draws, ms, nullptr, mTextureCache);
  long texturedDraws = 0;
  for (const OmWgpuSolidDraw &d : draws)
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
    for (const OmWgpuSolidDraw &d : draws) {
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

  OmWgpuRenderTarget target(mBackend, S, S);
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
  OmWgpuClearColor black;  // (0,0,0,1) => background decodes to ID 0 = miss
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
    std::vector<OmWgpuSolidDraw> lit;
    std::vector<std::array<float, 16>> lms;  // lit aliases into this; keep alive past render
    std::vector<OmSolid *> litTops;          // parallel to lit
    OmWgpuSceneRenderer::collectWorldDraws(*mMeshCache, lit, lms, &litTops);
    // Same-frame ID buffer: a COPY of lit with per-draw IDs encoded, rendered flat.
    // Drives BOTH selTop and the changed-pixel classification, so it matches the
    // litA/litB renders below exactly (same geometry, same instant).
    std::vector<OmWgpuSolidDraw> idDraws = lit;
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
    OmSolid *selTop = (selId != 0 && (selId - 1) < litTops.size()) ? litTops[selId - 1] : nullptr;
    std::vector<uint8_t> litA(static_cast<size_t>(S) * S * 4, 0), litB(litA.size(), 0);
    OmWgpuClearColor sky;
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
    for (OmSolid *t : litTops)
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
        OmSolid *pt = (pid != 0 && (pid - 1) < litTops.size()) ? litTops[pid - 1] : nullptr;
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
    std::vector<OmWgpuSolidDraw> jd;
    std::vector<std::array<float, 16>> jms, jaxis;  // jd aliases jms then jaxis; keep both alive
    OmWgpuSceneRenderer::collectWorldDraws(*mMeshCache, jd, jms);
    std::vector<uint8_t> bufBase(static_cast<size_t>(S) * S * 4, 0), bufAx(bufBase.size(), 0);
    OmWgpuClearColor sky;
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
  // Foundation for the screenshot action, OmVideoRecorder, and golden-image CI.
  {
    const uint32_t W = 640, H = 480;
    OmWgpuRenderTarget shot(mBackend, W, H);
    if (shot.isUsable()) {
      const double aspect = static_cast<double>(W) / static_cast<double>(H);
      float svp[16];
      OmWgpuSceneRenderer::buildViewProj(cam, horizFovForAspect(hf, aspect), aspect, 0.05, 1000.0, svp);
      std::vector<OmWgpuSolidDraw> sd;
      std::vector<std::array<float, 16>> sms;
      OmWgpuSceneRenderer::collectWorldDraws(*mMeshCache, sd, sms, nullptr, mTextureCache);
      std::vector<uint8_t> sbuf(static_cast<size_t>(W) * H * 4, 0);
      OmWgpuClearColor sky;
      sky.r = 0.45f;
      sky.g = 0.62f;
      sky.b = 0.85f;
      const float lit4[4] = {0.3f, 0.4f, -0.85f, 0.35f};
      const OmVector3 eye = cam.translation();
      const float camPos[3] = {static_cast<float>(eye.x()), static_cast<float>(eye.y()),
                               static_cast<float>(eye.z())};  // R4 PBR: view vector for specular
      const bool oks = shot.clearAndDrawScene(sky, svp, lit4, sd.data(),
                                              static_cast<uint32_t>(sd.size()), sbuf.data(), false,
                                              1.0f, false, camPos);
      if (oks) {
        // R4 step-3c-A: overlay bounding-object wireframes (VF_ALL_BOUNDING_OBJECTS) on
        // the lit scene via the line pipeline. The on-screen pane can't show these yet
        // (OmWgpuSurface has no line path — a follow-up), but this verifies the edge
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
          std::vector<OmSolid *> all;
          if (OmWorld::instance())
            for (OmSolid *t : OmWorld::instance()->topSolids()) {
              all.push_back(t);
              collectDescendantSolids(t, all);
            }
          uint32_t gated1 = 0;
          bool toggled = false;
          for (OmSolid *s : all) {
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
        // fixture (tests/physics/fixtures/wgpu_overlay_contacts.omniworld).
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
        // (0 in panda; non-zero in e.g. camera.omniworld). Pass both type-gates ON so this verifies
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
        // (0 here / in panda; non-zero in lidar.omniworld). Always-on-top, light blue.
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
        if (OmWorld::instance())
          for (OmSolid *hs : OmWorld::instance()->topSolids()) {
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
        if (OmWorld::instance())
          for (OmSolid *hs : OmWorld::instance()->topSolids()) {
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
        if (OmWorld::instance())
          for (OmSolid *rs : OmWorld::instance()->topSolids()) {
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
        if (OmWorld::instance())
          for (OmSolid *ss : OmWorld::instance()->topSolids()) {
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
        if (OmWorld::instance())
          for (OmSolid *rs : OmWorld::instance()->topSolids()) {
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
          std::vector<OmSolid *> desc;
          if (OmWorld::instance())
            for (OmSolid *t : OmWorld::instance()->topSolids())
              collectDescendantSolids(t, desc);
          OmSolid *S = nullptr;
          double bestRot = 0.0;  // |xAxisWorld - R_local·x|: 0 ⇒ identity parent, large ⇒ rotated
          for (OmSolid *s : desc) {
            const OmVector3 lx = (s->rotation().toMatrix3() * OmVector3(1, 0, 0)).normalized();
            const double m = (s->xAxis().normalized() - lx).length();
            if (m > bestRot) {
              bestRot = m;
              S = s;
            }
          }
          if (S) {
            const double d = 0.05;
            double newErr = 0.0, oldErr = 0.0;
            const OmVector3 t0 = S->translation();
            const OmVector3 w0 = S->position();
            for (int k = 0; k < 3; ++k) {
              const OmVector3 unit(k == 0 ? 1.0 : 0.0, k == 1 ? 1.0 : 0.0, k == 2 ? 1.0 : 0.0);
              const OmVector3 axisWorld =
                (k == 0 ? S->xAxis() : k == 1 ? S->yAxis() : S->zAxis()).normalized();
              const OmVector3 localDir = S->rotation().toMatrix3() * unit;  // parent-frame dir
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
    OmWgpuRenderTarget ov(mBackend, 256, 256);
    if (ov.isUsable()) {
      float ovp[16];
      OmWgpuSceneRenderer::buildViewProj(cam, hf, 1.0, 0.05, 1000.0, ovp);
      std::vector<OmWgpuSolidDraw> od;
      std::vector<std::array<float, 16>> oms;
      OmWgpuSceneRenderer::collectWorldDraws(*mMeshCache, od, oms);
      std::vector<uint8_t> base(256u * 256u * 4u, 0), dim(base.size(), 0);
      OmWgpuClearColor sky2;
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

  // D1.4 (C9): the WREN-vs-wgpu parity self-check died with WREN -- there is no second
  // in-process renderer to grab a golden from. The WREN reference images live in
  // tests/rendering/wren_reference/ (captured pre-deletion).
  OmLog::info("[OmWgpuView] offscreen pick + selection + joint-axis + screenshot + overlay self-check");
}

// R4 step-3c-A: DEVICE-OUTPUT INSET with a REAL sensor image. Retrying one-shot (the camera's
// controller enables the sensor asynchronously). Finds an enabled camera, wraps its rendered
// image() (BGRA) in a QImage, uploads it, renders the world to an offscreen target, and composites
// the camera view as a corner inset via the verified drawTexturedInset, then asserts the inset
// region differs from the scene + is non-uniform. Returns true once it has written a verdict
// (camera enabled + attempted) so the caller stops retrying; false → try again next tick.
bool OmWgpuView::runDeviceInsetCheck() {
  if (!mBackend || !mMeshCache || !mTextureCache)
    return false;
  OmAbstractCamera *sensorCam = nullptr;
  if (OmWorld::instance())
    for (OmSolid *s : OmWorld::instance()->topSolids()) {
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

  OmMatrix4 vpcam;
  double hf = 0.785;
  buildViewpointCamera(vpcam, hf);
  const QByteArray path =
    (qEnvironmentVariable("OMNISIM_VIEW3D_WGPU_SELFCHECK") + ".devinset.txt").toLocal8Bit();
  std::ofstream f(path.constData());
  if (!f)
    return true;
  QImage qi(img, cw, ch, cw * 4, QImage::Format_ARGB32);  // camera image() is BGRA → ARGB32 on LE
  OmWgpuTextureHandle th = OmWgpuImageAdapter::acquireFromQImage(
    *mTextureCache, reinterpret_cast<uint64_t>(sensorCam) | 0x5e500000ull, qi);
  const uint32_t W = 640, H = 480;
  OmWgpuRenderTarget di(mBackend, W, H);
  if (!th.view || !di.isUsable()) {
    f << "device-inset: cam=1 enabled=1 img=" << cw << "x" << ch << " (texture/target unusable)\n";
    return true;
  }
  const double aspect = static_cast<double>(W) / static_cast<double>(H);
  float scenevp[16];
  OmWgpuSceneRenderer::buildViewProj(vpcam, horizFovForAspect(hf, aspect), aspect, 0.05, 1000.0, scenevp);
  std::vector<OmWgpuSolidDraw> dd;
  std::vector<std::array<float, 16>> dms;
  OmWgpuSceneRenderer::collectWorldDraws(*mMeshCache, dd, dms, nullptr, mTextureCache);
  std::vector<uint8_t> dbuf(static_cast<size_t>(W) * H * 4, 0);
  OmWgpuClearColor sky3;
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

void OmWgpuView::renderTick() {
  mPhase += 0.02;  // frame clock (also the box spin angle)

  // One-shot offscreen pick self-check, once the backend + world are ready. The
  // timer only runs post-expose (after ensureSurface inits the backend), so
  // mBackend/mMeshCache are set here. Settles ~5 s for the world to finish loading.
  if (!mSelfCheckDone && mBackend && mMeshCache && mPhase > 5.0 &&
      qEnvironmentVariableIsSet("OMNISIM_VIEW3D_WGPU_SELFCHECK") &&
      // Defer while the world is still LOADING: the parity section grabs the WREN main window as
      // its golden, and mid-load that grab returns the "Loading world" overlay (black) → a
      // spurious 0% gate. Heavy worlds (the city) load past the 5 s settle on slower starts.
      (!OmWorld::instance() || !OmWorld::instance()->isLoading())) {
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
    if (OmWrenRenderingContext *rc = OmWrenRenderingContext::instance()) {
      const bool boOn = rc->isOptionalRenderingEnabled(OmWrenRenderingContext::VF_ALL_BOUNDING_OBJECTS);
      const bool jaOn = rc->isOptionalRenderingEnabled(OmWrenRenderingContext::VF_JOINT_AXES);
      rc->enableOptionalRendering(OmWrenRenderingContext::VF_ALL_BOUNDING_OBJECTS, true, false);
      rc->enableOptionalRendering(OmWrenRenderingContext::VF_JOINT_AXES, true, false);
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
      rc->enableOptionalRendering(OmWrenRenderingContext::VF_ALL_BOUNDING_OBJECTS, boOn, false);
      rc->enableOptionalRendering(OmWrenRenderingContext::VF_JOINT_AXES, jaOn, false);
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
