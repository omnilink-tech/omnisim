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

#include "OmApplication.hpp"
#include "OmGuiApplication.hpp"
#include "OmPhysicsBackend.hpp"
#include "OmLog.hpp"
#include "OmRenderBackend.hpp"
#include "OmVulkanBackend.hpp"
#include "OmNewtonBackend.hpp"
#include "OmPhysicsBackend.hpp"
#include "OmWgpuImageAdapter.hpp"
#include "OmWgpuMeshCache.hpp"
#include "OmWgpuRenderTarget.hpp"
#include "OmWgpuSceneRenderer.hpp"
#include "OmWgpuTextureCache.hpp"

#include <QtGui/QImage>

#include <QtCore/QFile>
#include <QtCore/QByteArray>

#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QLocale>
#include <QtCore/QRegularExpression>
#include <QtCore/QTextStream>
#include <QtCore/QVector>
#include <QtGui/QSurfaceFormat>
#include <QtWidgets/QApplication>

#include <algorithm>
#include <csignal>
#include <cstdlib>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#include <QtCore/QProcess>
extern "C" {
// defaults to nVidia instead of Intel graphics on Optimus architectures (commonly found on laptops)
// unfortunately, the AMD equivalent doesn't seem to exist.
__declspec(dllexport) DWORD NvOptimusEnablement = 0x00000001;
}
#ifdef NDEBUG
#include <stdio.h>
#include <wincon.h>
// What RedirectIOToConsole() decided, logged once the file log exists. The
// decision is invisible otherwise (this binary has no console to say it on),
// and it is the difference between an embedded interpreter whose sys.stdout
// works and one whose fd 1 is CLOSED -- see the freopen note below.
static QString gConsoleRedirectNote;

static void RedirectIOToConsole() {
  const HANDLE out = GetStdHandle(STD_OUTPUT_HANDLE);
  const DWORD file_type = (out == NULL || out == INVALID_HANDLE_VALUE) ? FILE_TYPE_UNKNOWN : GetFileType(out);
  // ⚠ ATTACH TO THE PARENT'S CONSOLE ONLY WHEN THE PARENT GAVE US NO STDOUT AT
  // ALL. Until 2026-08-29 the test was "not a pipe", so an engine handed the
  // null device (Popen(stdout=DEVNULL)) or a FILE (run-headless, the capture
  // service) threw that handle away and attached to whatever console its
  // launcher happened to have -- a console it does not own, shared with every
  // other engine and controller that launcher's console tree had spawned. Two
  // consequences, both measured with scripts/dev/launch_race_stress.py
  // --concurrent 2 --stagger 12 on machine 9722d23d12a3: the file the runner
  // gave us received NOTHING (every print went to the console instead), and in
  // 3 of 7 rounds the engine started against an already-running engine found
  // its fd 1 dead by the time the embedded interpreter first wrote to it
  // (os.fstat refused it in some, WriteFile returned ERROR_INVALID_HANDLE in
  // others) -- warp's greeting raised EBADF out of newton.ModelBuilder(), the
  // FFI smoke called the runtime broken, FATAL, exit 1. The same rounds with
  // stdout as a pipe (this branch) ran 8 of 8 clean. A GUI-subsystem binary
  // started from a console WITHOUT redirection gets no std handles at all
  // (they are NULL), which is the one case attaching was written for, and it
  // still takes it.
  if (file_type != FILE_TYPE_UNKNOWN) {
    gConsoleRedirectNote = QString("stdout inherited from the launcher (handle type %1: 1=file 2=char/null 3=pipe); "
                                   "stdio left as given, no console attached")
                             .arg(file_type);
    return;
  }
  if (!AttachConsole(ATTACH_PARENT_PROCESS)) {  // attempt to use the parent's console
    gConsoleRedirectNote = QString("no parent console to attach (AttachConsole error %1); stdio left as inherited "
                                   "(stdout handle type %2)")
                             .arg(GetLastError())
                             .arg(file_type);
    return;
  }
  // ⚠ freopen() CLOSES the stream before it tries to open the new file, so a
  // failed freopen("CONOUT$") leaves fd 1 closed -- and everything that later
  // writes to fd 1 gets EBADF, including the embedded Python interpreter's
  // sys.stdout, whose first real write (warp's greeting) then raises out of
  // newton.ModelBuilder() and takes the physics backend down. A stream that
  // cannot reach the console must land on the null device, never on nothing.
  const bool outOk = freopen("CONOUT$", "w", stdout) != nullptr;
  if (!outOk)
    (void)freopen("NUL", "w", stdout);
  const bool errOk = freopen("CONOUT$", "w", stderr) != nullptr;
  if (!errOk)
    (void)freopen("NUL", "w", stderr);
  const bool inOk = freopen("CONIN$", "r", stdin) != nullptr;
  if (!inOk)
    (void)freopen("NUL", "r", stdin);
  gConsoleRedirectNote = QString("attached to the parent console (stdout handle type %1); stdout=%2 stderr=%3 stdin=%4")
                           .arg(file_type)
                           .arg(outOk ? "CONOUT$" : "NUL (CONOUT$ refused)")
                           .arg(errOk ? "CONOUT$" : "NUL (CONOUT$ refused)")
                           .arg(inOk ? "CONIN$" : "NUL (CONIN$ refused)");
}
#endif
#else
#include <locale.h>
#endif

static QVector<QRegularExpression *> gQtMessageFilters;

// http://doc.qt.io/qt-5/qtglobal.html#qInstallMessageHandler
static void catchMessageOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg) {
  if (!gQtMessageFilters.isEmpty()) {
    QRegularExpressionMatch match;
    foreach (QRegularExpression *re, gQtMessageFilters) {
      match = re->match(msg);
      if (match.hasMatch())
        // filter out message
        return;
    }
  }

  QString message = msg;
  if (context.file != NULL)
    message += QString("(%s:%u, %s)").arg(context.file).arg(context.line).arg(context.function);
  switch (type) {
    case QtInfoMsg:
      fprintf(stderr, "Info: %s\n", message.toUtf8().constData());
      OmLog::fileLog("Qt Info: " + message);
      break;
    case QtDebugMsg:
      fprintf(stderr, "Debug: %s\n", message.toUtf8().constData());
      OmLog::fileLog("Qt Debug: " + message);
      break;
    case QtWarningMsg:
      fprintf(stderr, "Warning: %s\n", message.toUtf8().constData());
      OmLog::fileLog("Qt Warning: " + message);
      break;
    case QtCriticalMsg:
      fprintf(stderr, "Critical: %s\n", message.toUtf8().constData());
      OmLog::fileLog("Qt Critical: " + message);
      break;
    case QtFatalMsg:
      fprintf(stderr, "Fatal: %s\n", message.toUtf8().constData());
      OmLog::fileLog("Qt Fatal: " + message);
      abort();
  }
}

static void quitApplication(int sig) {
  OmApplication::instance()->simulationQuit(EXIT_SUCCESS);
}

int main(int argc, char *argv[]) {
#ifdef _WIN32
  // on Windows, the webots binary is located in $WEBOTS_HOME/msys64/mingw64/bin/webots
  // we need to use GetModuleFileName as argv[0] doesn't always provide an absolute path
  const int BUFFER_SIZE = 4096;
  wchar_t *tmp = new wchar_t[BUFFER_SIZE];
  GetModuleFileNameW(NULL, tmp, BUFFER_SIZE);
  const QString modulePath = QString::fromWCharArray(tmp);
  delete[] tmp;
  const QString omnisimDirPath = QDir(QFileInfo(modulePath).absolutePath() + "/../../..").canonicalPath();
#ifdef NDEBUG
  const char *MSYSCON = getenv("MSYSCON");
  if (MSYSCON && strncmp("mintty.exe", MSYSCON, 10) == 0)
    // if webots was started from a MINGW mintty console
    // we need to unbuffer the stderr as _IOLBF is not working in the msys console
    setvbuf(stderr, NULL, _IONBF, 0);
  else                      // started from a DOS console or from Windows (double click on Webots icon)
    RedirectIOToConsole();  // the release version is built with the -mwindows flag
                            // which drops stdout/stderr, so we need to redirect
                            // them to the parent console in case OmniSim was started
                            // from a DOS console
#else
  // we need to unbuffer the stderr as _IOLBF is not working in the msys console
  setvbuf(stderr, NULL, _IONBF, 0);
#endif  // NDEBUG
#elif defined(__linux__)
  // on Linux, the webots binary is located in $WEBOTS_HOME/bin/omnisim-bin
  const QString omnisimDirPath = QDir(QFileInfo(argv[0]).absolutePath() + "/..").canonicalPath();
#elif defined(__APPLE__)
  // on macOS, the webots binary is located in $WEBOTS_HOME/Contents/MacOS/webots
  const QString omnisimDirPath = QDir(QFileInfo(argv[0]).absolutePath() + "/../..").canonicalPath();
#endif
  QLocale::setDefault(QLocale::c());

  // Initialize file logging to WEBOTS_HOME/omnisim_log.txt
  OmLog::initFileLog(omnisimDirPath + "/omnisim_log.txt");
#if defined(_WIN32) && defined(NDEBUG)
  if (!gConsoleRedirectNote.isEmpty())
    OmLog::info("[main] stdio: " + gConsoleRedirectNote);
#endif

  // Begin the process-wide Newton import at the earliest safe point. There is
  // substantial renderer/Qt/application setup below before the first world
  // reaches physics construction; doing the import on the critical path there
  // costs several seconds. The load-world call repeats this idempotently so
  // non-GUI application entry points retain the same optimization.
  //
  // ⚠ EXCEPT FOR THE INFORMATIONAL TASKS, WHICH NEVER LOAD A WORLD AND MUST
  // NOT PAY FOR (OR WAIT ON) A PHYSICS RUNTIME. The preload runs on a
  // std::async shared state, and nothing joins it on a path that exits before
  // an OmApplication is destroyed -- so `omnisim-bin --version` HUNG
  // INDEFINITELY (measured 100 s, still running, exit 124) while holding TCP
  // 1234. The smoke runner spawns exactly that as its probe, so the whole
  // local-CI lane aborted with "TCP port 1234 is already in use -- likely a
  // running OmniSim GUI", pointing at a phantom zombie instead of at this.
  // Confirmed by the existing hatch: OMNISIM_NEWTON_ASYNC_PRELOAD=0 returns
  // --version to 0.36 s / exit 0.
  //
  // The flag spellings are the ones OmGuiApplication::parseArguments maps to
  // HELP / SYSINFO / VERSION; `convert` and `--update-world` are deliberately
  // NOT here -- they do touch a world.
  bool informationalTaskOnly = false;
  for (int i = 1; i < argc && !informationalTaskOnly; ++i)
    informationalTaskOnly = (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "--sysinfo") == 0 ||
                             strcmp(argv[i], "--version") == 0);
  if (!informationalTaskOnly)
    OmPhysicsBackendRegistry::startNewtonRuntimePreload();
  else
    // Same pre-scan, second consumer: the OmGuiApplication constructor skips the
    // GUI chrome (widget style, application font, stylesheet) for these
    // print-and-exit tasks. Expected 20-60 ms of the 0.36 s --version baseline
    // above; measured by the parent's --version N=10 timing A/B.
    OmGuiApplication::skipStartupChromeForInformationalTask();

  // PPM dump helper for R3.6 golden-image harness. PPM is the
  // simplest binary image format that round-trips lossless: 3-line
  // header + width*height*3 bytes of raw RGB. Reads in any tool
  // (PIL, ImageMagick, GIMP). We pick PPM over PNG so this binary
  // doesn't grow a libpng dep just for goldens.
  auto dumpPpm = [](const QString &dir, const QString &name, uint32_t w, uint32_t h,
                    const unsigned char *rgba) {
    if (dir.isEmpty())
      return;
    QFile f(dir + "/" + name + ".ppm");
    if (!f.open(QIODevice::WriteOnly))
      return;
    QByteArray hdr = QString("P6\n%1 %2\n255\n").arg(w).arg(h).toUtf8();
    f.write(hdr);
    QByteArray rgb(static_cast<int>(w * h * 3), 0);
    for (uint32_t i = 0; i < w * h; ++i) {
      rgb[3 * i + 0] = static_cast<char>(rgba[4 * i + 0]);
      rgb[3 * i + 1] = static_cast<char>(rgba[4 * i + 1]);
      rgb[3 * i + 2] = static_cast<char>(rgba[4 * i + 2]);
    }
    f.write(rgb);
  };
  const QString goldenDir = qEnvironmentVariable("OMNISIM_WGPU_PROBE_DIR");

  // R3 wgpu runtime smoke knob (engine-migration-plan.md §14.3).
  // Fires before any world load so it works in --no-rendering /
  // --batch / --minimize headless modes where the WREN backend
  // resolver never runs.
  //
  //   OMNISIM_PROBE_WGPU=1: probe R3.1 (init only). Forces
  //   OmVulkanBackend construction; isAvailable() flips true iff
  //   wgpu-native v29 successfully opens instance + adapter + device.
  //
  //   OMNISIM_PROBE_WGPU=2: probe R3.1+R3.3 end-to-end. Opens a
  //   8x8 RGBA8Unorm OmWgpuRenderTarget, calls clearAndRead(magenta),
  //   and checks the readback for the expected pixel value. The
  //   single byte we sample (pixel (0,0) red channel) is sufficient
  //   to prove the full pipeline (encode -> submit -> copy -> map
  //   -> readback) runs on this GPU.
  //
  // No-ops if OMNISIM_WITH_VULKAN=OFF or WGPU_NATIVE_HOME wasn't
  // set at build time — gVulkan ends up in the R1-style unavailable
  // branch and the probes log a single "wgpu-native unavailable" line.
  {
    // R4 step-3c-A.1 headless pick-probe: verify kSolidPick + clearAndDrawScene's
    // pickMode + the ID decode with NO GUI / window / exposure / timer dependency
    // (the GUI self-check could not be made to fire reliably). Renders a known
    // centre-covering triangle with an encoded ID=1 to an offscreen RGBA8 target,
    // reads back, and asserts centre decodes to 1 and a corner to 0 (background).
    // Run: OMNISIM_PROBE_PICK=<file> omnisim-bin --help   (on a wgpu-ON build).
    if (qEnvironmentVariableIsSet("OMNISIM_PROBE_PICK")) {
      QFile pf(qEnvironmentVariable("OMNISIM_PROBE_PICK"));
      if (pf.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QString res;
        OmRenderBackend *vulkan = OmRenderBackendRegistry::vulkanBackend();
        if (!vulkan || !vulkan->isAvailable()) {
          res = "FAIL wgpu-native unavailable (need a wgpu-ON build)\n";
        } else {
          OmVulkanBackend *vb = static_cast<OmVulkanBackend *>(vulkan);
          const uint32_t S = 64;
          OmWgpuRenderTarget rt(vb, S, S);
          OmWgpuMeshCache cache(vb);
          const float verts[24] = {
            -0.8f, -0.8f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
             0.8f, -0.8f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f,
             0.0f,  0.8f, 0.0f, 0.0f, 0.0f, 1.0f, 0.5f, 1.0f,
          };
          const uint32_t indices[3] = {0, 1, 2};
          OmWgpuMeshHandle mesh =
            cache.acquire(0xC0FFEEull, verts, sizeof(verts), indices, sizeof(indices), 3, 32);
          const float identity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
          OmWgpuSolidDraw d;
          d.modelMatrix16 = identity;
          d.baseColorR = 1.0f / 255.0f;  // encode draw ID = 1 (rgb bytes 1,0,0)
          d.baseColorG = 0.0f;
          d.baseColorB = 0.0f;
          d.baseColorA = 1.0f;
          d.vertexBuffer = mesh.vertexBuffer;
          d.indexBuffer = mesh.indexBuffer;
          d.indexCount = mesh.indexCount;
          unsigned char buf[64 * 64 * 4] = {};
          OmWgpuClearColor black;                            // (0,0,0,1) -> background id 0
          const float light[4] = {0.0f, 0.0f, -1.0f, 1.0f};  // ignored by kSolidPick
          const bool ok =
            rt.isUsable() && mesh.vertexBuffer &&
            rt.clearAndDrawScene(black, identity, light, &d, 1, buf, false, 1.0f, false, nullptr,
                                 /*pickMode=*/true);
          const int cc = (static_cast<int>(S / 2) * static_cast<int>(S) + static_cast<int>(S / 2)) * 4;
          const unsigned centerId = ok ? (buf[cc] | (buf[cc + 1] << 8) | (buf[cc + 2] << 16)) : 0u;
          const int kc = (4 * static_cast<int>(S) + 4) * 4;
          const unsigned cornerId = ok ? (buf[kc] | (buf[kc + 1] << 8) | (buf[kc + 2] << 16)) : 999u;
          res = QString("render_ok=%1\ncenter id=%2 (expect 1)  corner id=%3 (expect 0)\n%4\n")
                  .arg(ok ? 1 : 0)
                  .arg(centerId)
                  .arg(cornerId)
                  .arg((ok && centerId == 1 && cornerId == 0)
                         ? "PASS - wgpu pick verified (kSolidPick + pickMode + decode)"
                         : "FAIL");
        }
        pf.write(res.toUtf8());
        pf.close();
      }
    }

    // R4 robustness: readback-determinism probe. Renders ONE fixed lit scene N
    // times through clearAndDrawScene and checks every readback is byte-identical
    // to the first. Deterministically characterizes the intermittent offscreen-
    // readback corruption (self-check selection diff flips ~1/3 runs) with NO GUI.
    //   Run: OMNISIM_PROBE_READBACK=<file> omnisim-bin --help   (wgpu-ON build).
    if (qEnvironmentVariableIsSet("OMNISIM_PROBE_READBACK")) {
      QFile pf(qEnvironmentVariable("OMNISIM_PROBE_READBACK"));
      if (pf.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QString res;
        OmRenderBackend *vulkan = OmRenderBackendRegistry::vulkanBackend();
        if (!vulkan || !vulkan->isAvailable()) {
          res = "FAIL wgpu-native unavailable (need a wgpu-ON build)\n";
        } else {
          OmVulkanBackend *vb = static_cast<OmVulkanBackend *>(vulkan);
          const uint32_t S = 256;
          OmWgpuRenderTarget rt(vb, S, S);
          OmWgpuMeshCache cache(vb);
          const float verts[24] = {
            -0.8f, -0.8f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
             0.8f, -0.8f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f,
             0.0f,  0.8f, 0.0f, 0.0f, 0.0f, 1.0f, 0.5f, 1.0f,
          };
          const uint32_t indices[3] = {0, 1, 2};
          OmWgpuMeshHandle mesh =
            cache.acquire(0xBEEF01ull, verts, sizeof(verts), indices, sizeof(indices), 3, 32);
          const float identity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
          // A handful of draws so the pass is non-trivial (closer to a real scene).
          const int kDraws = 8;
          OmWgpuSolidDraw ds[kDraws];
          for (int k = 0; k < kDraws; ++k) {
            ds[k].modelMatrix16 = identity;
            ds[k].baseColorR = 0.2f + 0.08f * k;
            ds[k].baseColorG = 0.9f - 0.05f * k;
            ds[k].baseColorB = 0.3f;
            ds[k].baseColorA = 1.0f;
            ds[k].vertexBuffer = mesh.vertexBuffer;
            ds[k].indexBuffer = mesh.indexBuffer;
            ds[k].indexCount = mesh.indexCount;
          }
          const float light[4] = {0.3f, 0.4f, -0.85f, 0.35f};
          OmWgpuClearColor sky;
          sky.r = 0.45f;
          sky.g = 0.62f;
          sky.b = 0.85f;
          const size_t bytes = static_cast<size_t>(S) * S * 4;
          std::vector<unsigned char> buf0(bytes, 0), buf(bytes, 0);
          const int N = 64;
          int renderFails = 0, framesDiffering = 0;
          long maxByteDelta = 0;
          const bool ok0 =
            rt.isUsable() && mesh.vertexBuffer &&
            rt.clearAndDrawScene(sky, identity, light, ds, kDraws, buf0.data());
          for (int it = 1; ok0 && it < N; ++it) {
            std::fill(buf.begin(), buf.end(), 0);
            if (!rt.clearAndDrawScene(sky, identity, light, ds, kDraws, buf.data())) {
              ++renderFails;
              continue;
            }
            long delta = 0;
            for (size_t i = 0; i < bytes; ++i) {
              const int dd = std::abs(static_cast<int>(buf[i]) - static_cast<int>(buf0[i]));
              if (dd) {
                ++delta;
                if (dd > maxByteDelta)
                  maxByteDelta = dd;
              }
            }
            if (delta)
              ++framesDiffering;
          }
          res = QString("readback-stability: N=%1 kDraws=%2 render_ok0=%3 renderFails=%4 "
                        "framesDiffering=%5 maxByteDelta=%6\n%7\n")
                  .arg(N)
                  .arg(kDraws)
                  .arg(ok0 ? 1 : 0)
                  .arg(renderFails)
                  .arg(framesDiffering)
                  .arg(maxByteDelta)
                  .arg((ok0 && renderFails == 0 && framesDiffering == 0)
                         ? "PASS - readback is deterministic across N renders"
                         : "FAIL - readback non-deterministic (the race)");
        }
        pf.write(res.toUtf8());
        pf.close();
      }
    }

    // R4 3c-B UN-GATE leak-hunt: sustained-render SOAK probe. Runs the EXACT per-frame
    // offscreen submit+readback (clearAndDrawScene) that OOMs the gated main-view path after
    // ~2000 frames, in a tight headless loop with NO GUI/world, dumping wgpu-native's global
    // resource-registry counts (wgpuGenerateReport) every 100 frames. If it OOMs like the GUI, the
    // last lines pin WHICH resource type accumulates → app-level vs wgpu-native-internal, the open
    // question blocking the un-gate. If it survives clean with flat counts, the leak is below the
    // registry (driver/allocator) or scene/texture-dependent.
    //   Run: OMNISIM_PROBE_SOAK=<file> [OMNISIM_PROBE_SOAK_RES=1024] [OMNISIM_PROBE_SOAK_ITERS=6000]
    //        omnisim-bin --help   (wgpu-ON build; from PowerShell so Warp/Vulkan resolves)
    if (qEnvironmentVariableIsSet("OMNISIM_PROBE_SOAK")) {
      const QString sp = qEnvironmentVariable("OMNISIM_PROBE_SOAK");
      // Truncate up front so a mid-loop crash leaves only this run's lines.
      {
        QFile pf(sp);
        if (pf.open(QIODevice::WriteOnly | QIODevice::Text))
          pf.close();
      }
      auto append = [&sp](const QString &s) {
        QFile f(sp);
        if (f.open(QIODevice::Append | QIODevice::Text)) {
          f.write(s.toUtf8());
          f.close();
        }
      };
      OmRenderBackend *vulkan = OmRenderBackendRegistry::vulkanBackend();
      if (!vulkan || !vulkan->isAvailable()) {
        append("FAIL wgpu-native unavailable (need a wgpu-ON build)\n");
      } else {
        OmVulkanBackend *vb = static_cast<OmVulkanBackend *>(vulkan);
        int resReq = qEnvironmentVariableIsSet("OMNISIM_PROBE_SOAK_RES")
                       ? qEnvironmentVariable("OMNISIM_PROBE_SOAK_RES").toInt()
                       : 1024;
        if (resReq < 64)
          resReq = 64;
        const uint32_t S = static_cast<uint32_t>(resReq);
        const int iters = qEnvironmentVariableIsSet("OMNISIM_PROBE_SOAK_ITERS")
                            ? qEnvironmentVariable("OMNISIM_PROBE_SOAK_ITERS").toInt()
                            : 6000;
        append(QString("SOAK begin res=%1 iters=%2\n").arg(S).arg(iters));
        OmWgpuRenderTarget rt(vb, S, S);
        OmWgpuMeshCache cache(vb);
        const float verts[24] = {
          -0.8f, -0.8f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
           0.8f, -0.8f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f,
           0.0f,  0.8f, 0.0f, 0.0f, 0.0f, 1.0f, 0.5f, 1.0f,
        };
        const uint32_t indices[3] = {0, 1, 2};
        OmWgpuMeshHandle mesh =
          cache.acquire(0x50A4ull, verts, sizeof(verts), indices, sizeof(indices), 3, 32);
        const float identity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
        const int kDraws = 8;
        OmWgpuSolidDraw ds[kDraws];
        for (int k = 0; k < kDraws; ++k) {
          ds[k].modelMatrix16 = identity;
          ds[k].baseColorR = 0.2f + 0.08f * k;
          ds[k].baseColorG = 0.9f - 0.05f * k;
          ds[k].baseColorB = 0.3f;
          ds[k].baseColorA = 1.0f;
          ds[k].vertexBuffer = mesh.vertexBuffer;
          ds[k].indexBuffer = mesh.indexBuffer;
          ds[k].indexCount = mesh.indexCount;
        }
        const float light[4] = {0.3f, 0.4f, -0.85f, 0.35f};
        OmWgpuClearColor sky;
        sky.r = 0.45f;
        sky.g = 0.62f;
        sky.b = 0.85f;
        std::vector<unsigned char> buf(static_cast<size_t>(S) * S * 4, 0);
        const QByteArray spb = sp.toLocal8Bit();
        if (!rt.isUsable() || !mesh.vertexBuffer) {
          append("FAIL render target / mesh not usable\n");
        } else {
          int fails = 0;
          for (int it = 0; it < iters; ++it) {
            if (it % 100 == 0)
              rt.appendResourceReport(spb.constData(), it);
            if (!rt.clearAndDrawScene(sky, identity, light, ds, kDraws, buf.data()))
              ++fails;
          }
          rt.appendResourceReport(spb.constData(), iters);
          append(QString("SOAK survived: iters=%1 renderFails=%2\n").arg(iters).arg(fails));
        }
      }
    }

    // R4 step-3c-A line-pipeline probe. Draws a red cross (one horizontal + one
    // vertical segment through NDC origin) via clearAndDrawLines and confirms the
    // centre column/row are red and a corner is background — proves the LineList
    // pipeline that the optional renderings (bounding objects, normals, …) ride on.
    //   Run: OMNISIM_PROBE_LINE=<file> omnisim-bin --help   (wgpu-ON build).
    if (qEnvironmentVariableIsSet("OMNISIM_PROBE_LINE")) {
      QFile pf(qEnvironmentVariable("OMNISIM_PROBE_LINE"));
      if (pf.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QString res;
        OmRenderBackend *vulkan = OmRenderBackendRegistry::vulkanBackend();
        if (!vulkan || !vulkan->isAvailable()) {
          res = "FAIL wgpu-native unavailable (need a wgpu-ON build)\n";
        } else {
          OmVulkanBackend *vb = static_cast<OmVulkanBackend *>(vulkan);
          const uint32_t S = 128;
          OmWgpuRenderTarget rt(vb, S, S);
          const float identity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
          // 4 verts = 2 segments, pos3+normal3+uv2 (stride 32; normal/uv ignored).
          const float lv[32] = {
            -0.9f, 0.0f, 0.0f, 0, 0, 0, 0, 0,  0.9f, 0.0f, 0.0f, 0, 0, 0, 0, 0,
             0.0f, -0.9f, 0.0f, 0, 0, 0, 0, 0,  0.0f, 0.9f, 0.0f, 0, 0, 0, 0, 0,
          };
          const float red[4] = {1.0f, 0.0f, 0.0f, 1.0f};
          OmWgpuClearColor navy;
          navy.r = 0.0f;
          navy.g = 0.0f;
          navy.b = 0.2f;
          std::vector<unsigned char> buf(static_cast<size_t>(S) * S * 4, 0);
          const bool ok = rt.isUsable() && rt.clearAndDrawLines(navy, identity, red, lv, 4, buf.data());
          auto isRed = [&](int x, int y) {
            const size_t p = (static_cast<size_t>(y) * S + x) * 4;
            return buf[p] > 200 && buf[p + 1] < 60 && buf[p + 2] < 60;
          };
          // Total red across the frame: two full segments span ~2*S px; a collapsed
          // (degenerate) line would be ~1. Robust to the 1-px rasterization offset of
          // a line exactly on the NDC axis.
          long redTotal = 0;
          for (uint32_t y = 0; y < S; ++y)
            for (uint32_t x = 0; x < S; ++x)
              if (isRed(static_cast<int>(x), static_cast<int>(y)))
                ++redTotal;
          const bool cornerBg = ok && !isRed(3, 3);
          res = QString("line-pipeline: render_ok=%1 red-px=%2 (expect ~%3 for two spans) corner-bg=%4\n%5\n")
                  .arg(ok ? 1 : 0)
                  .arg(redTotal)
                  .arg(2 * S)
                  .arg(cornerBg ? 1 : 0)
                  .arg((ok && redTotal > static_cast<long>(S) && cornerBg)
                         ? "PASS - wgpu line pipeline draws colored segments"
                         : "FAIL");
        }
        pf.write(res.toUtf8());
        pf.close();
      }
    }

    // R4 material fidelity probe: build the textured-lit pipeline (naga-validates the
    // 4-binding WGSL: uniform + albedo + roughness + sampler), render a known textured
    // quad, and confirm (a) albedo is sampled (centre pixel pulled off white baseColor)
    // and (b) a black roughnessMap forces max specular (brighter than the white-roughness
    // render) — proving per-pixel roughness modulates the highlight. NO GUI/exposure.
    //   Run: OMNISIM_PROBE_TEX=<file> omnisim-bin --help   (wgpu-ON build).
    if (qEnvironmentVariableIsSet("OMNISIM_PROBE_TEX")) {
      QFile pf(qEnvironmentVariable("OMNISIM_PROBE_TEX"));
      if (pf.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QString res;
        OmRenderBackend *vulkan = OmRenderBackendRegistry::vulkanBackend();
        if (!vulkan || !vulkan->isAvailable()) {
          res = "FAIL wgpu-native unavailable (need a wgpu-ON build)\n";
        } else {
          OmVulkanBackend *vb = static_cast<OmVulkanBackend *>(vulkan);
          OmWgpuRenderTarget rt(vb, 64, 64);
          unsigned char alb[3] = {0, 0, 0}, spc[3] = {0, 0, 0}, mtl[3] = {0, 0, 0}, nrm[3] = {0, 0, 0};
          const bool built = rt.isUsable() && rt.selfTestTextured(alb, spc, mtl, nrm);
          // Albedo proof: centre is the textured (0.1,0.3,0.5)+specular colour, NOT the
          // flat-fallback white (255,255,255). Roughness proof: black-roughness render is
          // brighter (more specular) than the white-roughness render. Metalness proof: the
          // metal render tints its specular by the albedo (blue-dominant) with no diffuse, so
          // it stays albedo-coloured — distinctly NOT the white dielectric highlight. Normal
          // proof: a tilted normal map collapses the head-on highlight → dimmer than specOut.
          const int albSum = alb[0] + alb[1] + alb[2];
          const int spcSum = spc[0] + spc[1] + spc[2];
          const int mtlSum = mtl[0] + mtl[1] + mtl[2];
          const int nrmSum = nrm[0] + nrm[1] + nrm[2];
          const bool albedoSampled = built && alb[2] > alb[0] && albSum > 60 && albSum < 720;
          const bool roughModulates = built && spcSum > albSum;
          const bool metalTints = built && mtl[2] > mtl[0] + 20 && mtlSum < spcSum;
          const bool normalPerturbs = built && nrmSum < spcSum - 60;
          res = QString("scn-tex: pipeline_built=%1 albedo=(%2,%3,%4) blackRough=(%5,%6,%7) "
                        "metal=(%8,%9,%10) tiltNormal=(%11,%12,%13) albedoSampled=%14 "
                        "roughModulates=%15 metalTints=%16 normalPerturbs=%17\n%18\n")
                  .arg(built ? 1 : 0)
                  .arg(alb[0]).arg(alb[1]).arg(alb[2])
                  .arg(spc[0]).arg(spc[1]).arg(spc[2])
                  .arg(mtl[0]).arg(mtl[1]).arg(mtl[2])
                  .arg(nrm[0]).arg(nrm[1]).arg(nrm[2])
                  .arg(albedoSampled ? 1 : 0)
                  .arg(roughModulates ? 1 : 0)
                  .arg(metalTints ? 1 : 0)
                  .arg(normalPerturbs ? 1 : 0)
                  .arg((built && albedoSampled && roughModulates && metalTints && normalPerturbs)
                         ? "PASS - 6-binding pipeline: albedo + roughness + metalness + normalMap"
                         : "FAIL");
        }
        pf.write(res.toUtf8());
        pf.close();
      }
    }

    // R4 device-inset probe: clear a target to navy, composite a known RED texture into a
    // top-right NDC sub-rect via the screen-space textured-quad primitive (the device-output
    // inset mechanism), and confirm a pixel INSIDE the inset is red while one OUTSIDE stays
    // navy. NO GUI/exposure. Run: OMNISIM_PROBE_INSET=<file> omnisim-bin --help (wgpu-ON build).
    if (qEnvironmentVariableIsSet("OMNISIM_PROBE_INSET")) {
      QFile pf(qEnvironmentVariable("OMNISIM_PROBE_INSET"));
      if (pf.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QString res;
        OmRenderBackend *vulkan = OmRenderBackendRegistry::vulkanBackend();
        if (!vulkan || !vulkan->isAvailable()) {
          res = "FAIL wgpu-native unavailable (need a wgpu-ON build)\n";
        } else {
          OmVulkanBackend *vb = static_cast<OmVulkanBackend *>(vulkan);
          OmWgpuRenderTarget rt(vb, 64, 64);
          unsigned char in[3] = {0, 0, 0}, out[3] = {0, 0, 0};
          const bool ok = rt.isUsable() && rt.selfTestInset(in, out);
          const bool insideRed = ok && in[0] > 200 && in[1] < 70 && in[2] < 70;
          const bool outsideNavy = ok && out[0] < 70 && out[1] < 70 && out[2] > 25 && out[2] < 110;
          res = QString("tex-quad-inset: ok=%1 inside=(%2,%3,%4) outside=(%5,%6,%7) "
                        "insideRed=%8 outsideNavy=%9\n%10\n")
                  .arg(ok ? 1 : 0)
                  .arg(in[0]).arg(in[1]).arg(in[2])
                  .arg(out[0]).arg(out[1]).arg(out[2])
                  .arg(insideRed ? 1 : 0)
                  .arg(outsideNavy ? 1 : 0)
                  .arg((ok && insideRed && outsideNavy)
                         ? "PASS - screen-space textured inset composites into an NDC sub-rect"
                         : "FAIL");
        }
        pf.write(res.toUtf8());
        pf.close();
      }
    }

    // R4 lighting rung 1: build the textured+shadowed pipeline (naga-validates kSolidLitTexturedShadow
    // + its 9-entry material+shadow+light layout). NO render. Run: OMNISIM_PROBE_TEXSHADOW=<file> --help
    if (qEnvironmentVariableIsSet("OMNISIM_PROBE_TEXSHADOW")) {
      QFile pf(qEnvironmentVariable("OMNISIM_PROBE_TEXSHADOW"));
      if (pf.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QString res;
        OmRenderBackend *vulkan = OmRenderBackendRegistry::vulkanBackend();
        if (!vulkan || !vulkan->isAvailable()) {
          res = "FAIL wgpu-native unavailable (need a wgpu-ON build)\n";
        } else {
          OmVulkanBackend *vb = static_cast<OmVulkanBackend *>(vulkan);
          OmWgpuRenderTarget rt(vb, 64, 64);
          const bool built = rt.isUsable() && rt.probeTexturedShadowPipeline();
          res = QString("tex-shadow-pipeline: built=%1\n%2\n")
                  .arg(built ? 1 : 0)
                  .arg(built ? "PASS - kSolidLitTexturedShadow (9-binding material+shadow) naga-validates"
                             : "FAIL");
        }
        pf.write(res.toUtf8());
        pf.close();
      }
    }

    // T1.2 CSM (multi-cascade) probe: build the kSolidLitCsm pipeline (naga-validates the 3-entry
    // CsmScene uniform + texture_2d_array + non-filtering sampler layout IN-ENGINE), render the
    // GPU-proven prototype scene (a large floor + an elevated caster) through the N+1-pass CSM path
    // at strength 0 then 0.8, and confirm (a) the floor under the caster DARKENS, (b) a floor point
    // to the side STAYS LIT, (c) the shadow point routes through a NON-zero cascade (multi-cascade
    // selection genuinely exercised). NO GUI/exposure. The in-engine counterpart of
    // docs/developer/csm_render_prototype.py. Run: OMNISIM_PROBE_CSM=<file> omnisim-bin --help (wgpu-ON).
    if (qEnvironmentVariableIsSet("OMNISIM_PROBE_CSM")) {
      QFile pf(qEnvironmentVariable("OMNISIM_PROBE_CSM"));
      if (pf.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QString res;
        OmRenderBackend *vulkan = OmRenderBackendRegistry::vulkanBackend();
        if (!vulkan || !vulkan->isAvailable()) {
          res = "FAIL wgpu-native unavailable (need a wgpu-ON build)\n";
        } else {
          OmVulkanBackend *vb = static_cast<OmVulkanBackend *>(vulkan);
          OmWgpuRenderTarget rt(vb, 256, 256);
          unsigned char shd[3] = {0, 0, 0}, lit[3] = {0, 0, 0}, off[3] = {0, 0, 0};
          int cascade = -1;
          const bool built =
            rt.isUsable() && OmWgpuSceneRenderer::csmSelfTest(rt, shd, lit, off, &cascade);
          const int shdSum = shd[0] + shd[1] + shd[2];
          const int litSum = lit[0] + lit[1] + lit[2];
          const int offSum = off[0] + off[1] + off[2];
          const bool floorVisible = built && offSum > 60;            // floor renders lit when shadows off
          const bool casterShadows = built && shdSum < offSum * 0.85;  // under-caster darkens with shadows on
          const bool sideStaysLit = built && litSum > offSum * 0.85;   // side floor unshadowed (no false shadow)
          const bool multiCascade = built && cascade >= 1;            // shadow routes through a non-zero cascade
          res = QString("csm: built=%1 floorLit=(%2,%3,%4) shadowed=(%5,%6,%7) litSide=(%8,%9,%10) "
                        "cascade=%11 floorVisible=%12 casterShadows=%13 sideStaysLit=%14 multiCascade=%15\n%16\n")
                  .arg(built ? 1 : 0)
                  .arg(off[0]).arg(off[1]).arg(off[2])
                  .arg(shd[0]).arg(shd[1]).arg(shd[2])
                  .arg(lit[0]).arg(lit[1]).arg(lit[2])
                  .arg(cascade)
                  .arg(floorVisible ? 1 : 0)
                  .arg(casterShadows ? 1 : 0)
                  .arg(sideStaysLit ? 1 : 0)
                  .arg(multiCascade ? 1 : 0)
                  .arg((built && floorVisible && casterShadows && sideStaysLit && multiCascade)
                         ? "PASS - N-cascade shadow array + kSolidLitCsm: caster casts, side stays lit, "
                           "multi-cascade selected"
                         : "FAIL");
        }
        pf.write(res.toUtf8());
        pf.close();
      }
    }

    // T1.4 TAA probe: build the kTaaResolve pipeline (naga-validates the 4-entry TaaParams + cur +
    // hist + sampler layout IN-ENGINE) and run the temporal resolve over known uniform textures to
    // confirm (a) the feedback blend (cur white + hist black, fb 0.9 -> ~26), (b) the 3x3
    // neighborhood clamp suppresses an out-of-range history (cur black, hist white: clamp ON -> ~0
    // vs OFF -> ~230), (c) TAA-off passes the current frame through (~255), (d) off-screen history
    // is rejected (~255). NO GUI/exposure. The in-engine counterpart of taa-preview.html's resolve.
    //   Run: OMNISIM_PROBE_TAA=<file> omnisim-bin --help (wgpu-ON build).
    if (qEnvironmentVariableIsSet("OMNISIM_PROBE_TAA")) {
      QFile pf(qEnvironmentVariable("OMNISIM_PROBE_TAA"));
      if (pf.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QString res;
        OmRenderBackend *vulkan = OmRenderBackendRegistry::vulkanBackend();
        if (!vulkan || !vulkan->isAvailable()) {
          res = "FAIL wgpu-native unavailable (need a wgpu-ON build)\n";
        } else {
          OmVulkanBackend *vb = static_cast<OmVulkanBackend *>(vulkan);
          OmWgpuRenderTarget rt(vb, 64, 64);
          unsigned char bl[3] = {0, 0, 0}, cOn[3] = {0, 0, 0}, cOff[3] = {0, 0, 0}, off[3] = {0, 0, 0},
                        osc[3] = {0, 0, 0};
          const bool built = rt.isUsable() && rt.selfTestTaa(bl, cOn, cOff, off, osc);
          const bool blendBlends = built && bl[0] > 8 && bl[0] < 60;  // mix(white,black,0.9)=0.1 -> ~26
          const bool clampSuppresses =
            built && cOn[0] < 40 && cOff[0] > 180 && cOff[0] > cOn[0] + 120;  // clamp kills the ghost
          const bool taaOffPasses = built && off[0] > 230;       // TAA off -> cur passthrough (white)
          const bool offscreenRejected = built && osc[0] > 230;  // off-screen history dropped -> cur
          // Ping-pong history accumulator: seed black, accumulate white -> the EMA converges to white.
          unsigned char accR[3] = {0, 0, 0}, accF[3] = {0, 0, 0}, accM[3] = {0, 0, 0};
          const bool accBuilt = built && rt.selfTestTaaAccum(accR, accF, accM);
          const bool converges = accBuilt && accR[0] < 15 && accF[0] > accR[0] + 25 &&
                                 accM[0] > accF[0] + 40 && accM[0] > 230;  // monotone rise to ~white
          const bool allPass = built && blendBlends && clampSuppresses && taaOffPasses &&
                               offscreenRejected && converges;
          res = QString("taa: built=%1 blend=(%2,%3,%4) clampOn=(%5,%6,%7) clampOff=(%8,%9,%10) "
                        "taaOff=(%11,%12,%13) offscreen=(%14,%15,%16) accum[reset/few/many]=%17/%18/%19 "
                        "blendBlends=%20 clampSuppresses=%21 taaOffPasses=%22 offscreenRejected=%23 "
                        "converges=%24\n%25\n")
                  .arg(built ? 1 : 0)
                  .arg(bl[0]).arg(bl[1]).arg(bl[2])
                  .arg(cOn[0]).arg(cOn[1]).arg(cOn[2])
                  .arg(cOff[0]).arg(cOff[1]).arg(cOff[2])
                  .arg(off[0]).arg(off[1]).arg(off[2])
                  .arg(osc[0]).arg(osc[1]).arg(osc[2])
                  .arg(accR[0]).arg(accF[0]).arg(accM[0])
                  .arg(blendBlends ? 1 : 0)
                  .arg(clampSuppresses ? 1 : 0)
                  .arg(taaOffPasses ? 1 : 0)
                  .arg(offscreenRejected ? 1 : 0)
                  .arg(converges ? 1 : 0)
                  .arg(allPass
                         ? "PASS - kTaaResolve + ping-pong history: blend + clamp + guards + converges"
                         : "FAIL");
        }
        pf.write(res.toUtf8());
        pf.close();
      }
    }

    // T1.4 TAA jitter probe (GPU-FREE — pure CPU math): verify the Halton(2,3) 8-frame jitter sequence
    // stays within +/-amplitude and is well spread across the pixel, and that jitterViewProj applies a
    // PIXEL-ACCURATE clip-space shift (identity VP, world origin -> screen center; jitter +4px ->
    // center+4px in x). No GPU/backend needed. Run: OMNISIM_PROBE_TAA_JITTER=<file> omnisim-bin --help.
    if (qEnvironmentVariableIsSet("OMNISIM_PROBE_TAA_JITTER")) {
      QFile pf(qEnvironmentVariable("OMNISIM_PROBE_TAA_JITTER"));
      if (pf.open(QIODevice::WriteOnly | QIODevice::Text)) {
        const double amp = 0.5;
        float minx = 1.0e9f, maxx = -1.0e9f;
        bool inRange = true;
        for (int f = 0; f < 8; ++f) {
          float off[2] = {0.0f, 0.0f};
          OmWgpuSceneRenderer::haltonJitter(f, amp, off);
          const float ax = off[0] < 0 ? -off[0] : off[0];
          const float ay = off[1] < 0 ? -off[1] : off[1];
          if (ax > amp + 1e-4 || ay > amp + 1e-4)
            inRange = false;
          if (off[0] < minx) minx = off[0];
          if (off[0] > maxx) maxx = off[0];
        }
        const float spread = maxx - minx;
        const bool distributed = spread > static_cast<float>(amp);  // samples cover the pixel
        const double W = 64.0, H = 64.0;
        float vp[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};  // identity VP
        const float jpx[2] = {4.0f, 0.0f};
        OmWgpuSceneRenderer::jitterViewProj(vp, jpx, W, H);
        const double cx = vp[12];  // viewProj * (0,0,0,1): clip.x
        const double cw = vp[15];  // clip.w
        const double ndcx = cw != 0.0 ? cx / cw : 0.0;
        const double pxx = (ndcx * 0.5 + 0.5) * W;
        const double shiftPx = pxx - W / 2.0;  // expect ~4
        const double shiftErr = shiftPx - 4.0;
        const bool pixelAccurate = (shiftErr < 0.5 && shiftErr > -0.5);
        const bool pass = inRange && distributed && pixelAccurate;
        const QString res =
          QString("taa-jitter: inRange=%1 distributed=%2 spreadPx=%3 shiftPx=%4 pixelAccurate=%5\n%6\n")
            .arg(inRange ? 1 : 0)
            .arg(distributed ? 1 : 0)
            .arg(spread, 0, 'f', 3)
            .arg(shiftPx, 0, 'f', 3)
            .arg(pixelAccurate ? 1 : 0)
            .arg(pass ? "PASS - Halton(2,3) jitter sequence + pixel-accurate viewProj jitter" : "FAIL");
        pf.write(res.toUtf8());
        pf.close();
      }
    }

    // T1.3 fog probe: build the kFogResolve pipeline (naga-validates the 4-entry FogParams + scene +
    // depth + sampler layout IN-ENGINE) and run the distance-fog resolve over a white scene at a NEAR
    // vs FAR depth: the far pixel must be heavily fog-coloured (blue), the near ~scene (white), and
    // fog-off passes the scene through. NO GUI/exposure. Run: OMNISIM_PROBE_FOG=<file> omnisim-bin --help.
    if (qEnvironmentVariableIsSet("OMNISIM_PROBE_FOG")) {
      QFile pf(qEnvironmentVariable("OMNISIM_PROBE_FOG"));
      if (pf.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QString res;
        OmRenderBackend *vulkan = OmRenderBackendRegistry::vulkanBackend();
        if (!vulkan || !vulkan->isAvailable()) {
          res = "FAIL wgpu-native unavailable (need a wgpu-ON build)\n";
        } else {
          OmVulkanBackend *vb = static_cast<OmVulkanBackend *>(vulkan);
          OmWgpuRenderTarget rt(vb, 64, 64);
          unsigned char nr[3] = {0, 0, 0}, fr[3] = {0, 0, 0}, of[3] = {0, 0, 0};
          const bool built = rt.isUsable() && rt.selfTestFog(nr, fr, of);
          const bool nearClear = built && nr[0] > 230;                  // dist 1m -> ~white
          const bool farFogged = built && fr[0] < 100 && fr[2] > 150;   // dist 100m -> blue-dominant fog
          const bool fogWithDistance = built && (nr[0] - fr[0]) > 120;  // far is more fogged than near
          const bool offPasses = built && of[0] > 230;                  // fog off -> scene passthrough
          res = QString("fog: built=%1 near=(%2,%3,%4) far=(%5,%6,%7) off=(%8,%9,%10) nearClear=%11 "
                        "farFogged=%12 fogWithDistance=%13 offPasses=%14\n%15\n")
                  .arg(built ? 1 : 0)
                  .arg(nr[0]).arg(nr[1]).arg(nr[2])
                  .arg(fr[0]).arg(fr[1]).arg(fr[2])
                  .arg(of[0]).arg(of[1]).arg(of[2])
                  .arg(nearClear ? 1 : 0)
                  .arg(farFogged ? 1 : 0)
                  .arg(fogWithDistance ? 1 : 0)
                  .arg(offPasses ? 1 : 0)
                  .arg((built && nearClear && farFogged && fogWithDistance && offPasses)
                         ? "PASS - kFogResolve: exponential distance fog (far fogs, near clear, off passes)"
                         : "FAIL");
        }
        pf.write(res.toUtf8());
        pf.close();
      }
    }

    // OMNISIM_PROBE_BACKENDS -- headless backend-resolution probe (architectural-baseline.md §5
    // reversibility lever). Reports which physics + render backends the registries RESOLVE to under the
    // CURRENT environment, so reversibility_check.py can assert that OMNISIM_LEGACY=1 (and the per-arm
    // OMNISIM_FORCE_ODE / OMNISIM_FORCE_WREN) revert the whole process to ODE+WREN. Pure resolve() calls
    // through the two registries -- no world, no window; resolve() only touches GPU/Python if the env
    // actually selects the new backend (it does not under LEGACY/FORCE). Run via --help, like the others.
    if (qEnvironmentVariableIsSet("OMNISIM_PROBE_BACKENDS")) {
      QFile pf(qEnvironmentVariable("OMNISIM_PROBE_BACKENDS"));
      if (pf.open(QIODevice::WriteOnly | QIODevice::Text)) {
        OmRenderBackendRegistry::initialise();
        OmPhysicsBackendRegistry::initialise();
        auto rkind = [](OmRenderBackend *b) -> const char * {
          if (!b)
            return "null";
          switch (b->kind()) {
            case OmRenderBackendKind::Wren:
              return "Wren";
            case OmRenderBackendKind::Vulkan:
              return "Vulkan";
            default:
              return "Unknown";
          }
        };
        auto pkind = [](OmPhysicsBackend *b) -> const char * {
          if (!b)
            return "null";
          switch (b->kind()) {
            case OmPhysicsBackendKind::Ode:
              return "Ode";
            case OmPhysicsBackendKind::Newton:
              return "Newton";
            case OmPhysicsBackendKind::Auto:
              return "Auto";
            default:
              return "Unknown";
          }
        };
        // Render first + FLUSH: resolving render is cheap (wgpu device probe ~100ms), but resolving
        // physics may init Newton (embedded Python + warp kernel compile, tens of seconds on a cold
        // cache) -- so capture the render verdict before that, so a slow/killed physics resolve never
        // loses the render result. (F1: FORCE_WREN/LEGACY are retired warned no-ops and no longer
        // skip the wgpu probe.)
        OmRenderBackend *const r = OmRenderBackendRegistry::resolve(OmRenderBackendKind::Vulkan);
        pf.write(QString("render=%1 kind=%2 available=%3\n")
                   .arg(r ? r->name() : "null")
                   .arg(rkind(r))
                   .arg(r && r->isAvailable() ? 1 : 0)
                   .toUtf8());
        pf.flush();
        OmPhysicsBackend *const p = OmPhysicsBackendRegistry::resolve(OmPhysicsBackendKind::Newton);
        pf.write(QString("physics=%1 kind=%2 available=%3\n")
                   .arg(p ? p->name() : "null")
                   .arg(pkind(p))
                   .arg(p && p->isAvailable() ? 1 : 0)
                   .toUtf8());
        pf.close();
      }
    }

    const int probe = qEnvironmentVariableIntValue("OMNISIM_PROBE_WGPU");
    if (probe >= 1) {
      OmRenderBackend *vulkan = OmRenderBackendRegistry::vulkanBackend();
      if (!vulkan || !vulkan->isAvailable()) {
        OmLog::info("[OmWgpuBackend] wgpu-native unavailable (build flag off, dep missing, or "
                    "runtime init failed). Probe stops here.");
      } else if (probe >= 2) {
        OmVulkanBackend *vb = static_cast<OmVulkanBackend *>(vulkan);
        OmWgpuRenderTarget rt(vb, 8, 8);
        if (!rt.isUsable()) {
          OmLog::info("[OmWgpuRenderTarget] target construction failed; probe=2 stops here.");
        } else {
          OmWgpuClearColor color = {1.0f, 0.0f, 1.0f, 1.0f};  // magenta
          unsigned char pixels[8 * 8 * 4] = {};
          if (!rt.clearAndRead(color, pixels)) {
            OmLog::info("[OmWgpuRenderTarget] clearAndRead returned false; probe=2 fails.");
          } else {
            dumpPpm(goldenDir, "probe2_clear_magenta", 8, 8, pixels);
            const unsigned r = pixels[0];
            const unsigned g = pixels[1];
            const unsigned b = pixels[2];
            const unsigned a = pixels[3];
            const bool ok = (r >= 0xfd) && (g <= 0x02) && (b >= 0xfd) && (a >= 0xfd);
            OmLog::info(QString("[OmWgpuRenderTarget] clearAndRead OK; pixel(0,0)=(%1,%2,%3,%4) "
                                "expected ~(0xFF,0x00,0xFF,0xFF) %5")
                          .arg(r)
                          .arg(g)
                          .arg(b)
                          .arg(a)
                          .arg(ok ? "PASS" : "MISMATCH"));
          }
          if (probe >= 3) {
            // R3.4-step-1 probe: clear to black, draw the WGSL vertex-
            // shaded triangle, sample the center pixel. The center
            // pixel sits inside the triangle (NDC (0,0) for an 8x8
            // target) and should read back the barycentric blend ~=
            // (85, 85, 85, 255). We check (>=20) on all three channels
            // to confirm color came from the fragment shader, not the
            // black clear.
            unsigned char tpixels[8 * 8 * 4] = {};
            OmWgpuClearColor blk = {0.0f, 0.0f, 0.0f, 1.0f};
            if (!rt.clearAndDrawTriangle(blk, tpixels)) {
              OmLog::info("[OmWgpuRenderTarget] clearAndDrawTriangle returned false; probe=3 fails.");
            } else {
              dumpPpm(goldenDir, "probe3_triangle_bary", 8, 8, tpixels);
              const int cx = 4, cy = 4;
              const int idx = (cy * 8 + cx) * 4;
              const unsigned r2 = tpixels[idx + 0];
              const unsigned g2 = tpixels[idx + 1];
              const unsigned b2 = tpixels[idx + 2];
              const unsigned a2 = tpixels[idx + 3];
              const bool ok2 = (r2 >= 20) && (g2 >= 20) && (b2 >= 20) && (a2 >= 0xfd);
              OmLog::info(QString("[OmWgpuRenderTarget] clearAndDrawTriangle OK; "
                                  "pixel(4,4)=(%1,%2,%3,%4) expected ~bary blend %5")
                            .arg(r2)
                            .arg(g2)
                            .arg(b2)
                            .arg(a2)
                            .arg(ok2 ? "PASS" : "MISMATCH"));
            }
          }
          if (probe >= 4) {
            // R3.4-step-2 + R3.2 runtime probe: upload a triangle
            // through OmWgpuMeshCache in the production pos3+norm3+uv2
            // (32-byte stride) layout, then draw it through the
            // vertex-buffer-fed WGSL pipeline. Fragment colors by
            // the normal so the center pixel should be ~bluish
            // (normal = +Z = (0,0,1) on the three vertices).
            OmWgpuMeshCache cache(vb);
            // 3 verts * (pos3 + norm3 + uv2) = 3 * 32 = 96 bytes
            const float verts[24] = {
              -0.8f, -0.8f, 0.0f,  0.0f, 0.0f, 1.0f,  0.0f, 0.0f,
               0.8f, -0.8f, 0.0f,  0.0f, 0.0f, 1.0f,  1.0f, 0.0f,
               0.0f,  0.8f, 0.0f,  0.0f, 0.0f, 1.0f,  0.5f, 1.0f,
            };
            const uint32_t indices[3] = {0, 1, 2};
            OmWgpuMeshHandle mesh = cache.acquire(/*meshId*/ 0xDEADBEEF, verts, sizeof(verts),
                                                  indices, sizeof(indices), 3, 32);
            if (!mesh.vertexBuffer || !mesh.indexBuffer) {
              OmLog::info("[OmWgpuMeshCache] acquire failed; probe=4 stops here.");
            } else {
              unsigned char mpixels[8 * 8 * 4] = {};
              OmWgpuClearColor blk = {0.0f, 0.0f, 0.0f, 1.0f};
              if (!rt.clearAndDrawMesh(blk, mesh.vertexBuffer, mesh.indexBuffer, mesh.indexCount,
                                       mpixels)) {
                OmLog::info("[OmWgpuRenderTarget] clearAndDrawMesh returned false; probe=4 fails.");
              } else {
                dumpPpm(goldenDir, "probe4_mesh_normal", 8, 8, mpixels);
                const int idx = (4 * 8 + 4) * 4;
                const unsigned r3 = mpixels[idx + 0];
                const unsigned g3 = mpixels[idx + 1];
                const unsigned b3 = mpixels[idx + 2];
                const unsigned a3 = mpixels[idx + 3];
                // normal=(0,0,1) -> RGB=(0,0,1) -> readback (~0, ~0, 255).
                // R+G stay near 0; B near 255.
                const bool ok3 = (r3 < 20) && (g3 < 20) && (b3 >= 200) && (a3 >= 0xfd);
                OmLog::info(QString("[OmWgpuRenderTarget] clearAndDrawMesh OK; "
                                    "pixel(4,4)=(%1,%2,%3,%4) expected (~0,~0,~255,~255) %5")
                              .arg(r3)
                              .arg(g3)
                              .arg(b3)
                              .arg(a3)
                              .arg(ok3 ? "PASS" : "MISMATCH"));
              }
              if (probe >= 5) {
                // R3.4-step-3 probe: same mesh, same RT, but now route
                // through the MVP-uniform pipeline. The 4x4 viewProj
                // is identity (column-major: e0=(1,0,0,0), e1=(0,1,0,0),
                // e2=(0,0,1,0), e3=(0,0,0,1)) — same NDC positions as
                // the no-uniform path, so the readback at (4,4) must
                // match probe=4 (~0,~0,~255,~255).
                const float identity[16] = {
                  1, 0, 0, 0,
                  0, 1, 0, 0,
                  0, 0, 1, 0,
                  0, 0, 0, 1,
                };
                unsigned char upixels[8 * 8 * 4] = {};
                OmWgpuClearColor blk2 = {0.0f, 0.0f, 0.0f, 1.0f};
                if (!rt.clearAndDrawMeshMVP(blk2, identity, mesh.vertexBuffer, mesh.indexBuffer,
                                            mesh.indexCount, upixels)) {
                  OmLog::info("[OmWgpuRenderTarget] clearAndDrawMeshMVP returned false; "
                              "probe=5 fails.");
                } else {
                  dumpPpm(goldenDir, "probe5_mesh_mvp", 8, 8, upixels);
                  const int idx5 = (4 * 8 + 4) * 4;
                  const unsigned r5 = upixels[idx5 + 0];
                  const unsigned g5 = upixels[idx5 + 1];
                  const unsigned b5 = upixels[idx5 + 2];
                  const unsigned a5 = upixels[idx5 + 3];
                  const bool ok5 = (r5 < 20) && (g5 < 20) && (b5 >= 200) && (a5 >= 0xfd);
                  OmLog::info(QString("[OmWgpuRenderTarget] clearAndDrawMeshMVP OK; "
                                      "pixel(4,4)=(%1,%2,%3,%4) expected (~0,~0,~255,~255) %5")
                                .arg(r5)
                                .arg(g5)
                                .arg(b5)
                                .arg(a5)
                                .arg(ok5 ? "PASS" : "MISMATCH"));
                }
                if (probe >= 6) {
                  // R3.5 probe: upload a 2x2 solid-red RGBA texture
                  // through OmWgpuTextureCache, draw the production-
                  // layout triangle through the textured WGSL
                  // pipeline. Identity MVP keeps positions intact.
                  // Center pixel should sample the red texel, so
                  // readback at (4,4) ~= (255, 0, 0, 255).
                  OmWgpuTextureCache tcache(vb);
                  const unsigned char texBytes[2 * 2 * 4] = {
                    255, 0, 0, 255,  255, 0, 0, 255,
                    255, 0, 0, 255,  255, 0, 0, 255,
                  };
                  OmWgpuTextureHandle th =
                      tcache.acquire(0xC0FFEE, 2, 2, texBytes, sizeof(texBytes));
                  if (!th.view) {
                    OmLog::info("[OmWgpuTextureCache] acquire failed; probe=6 stops here.");
                  } else {
                    const float identity6[16] = {1, 0, 0, 0, 0, 1, 0, 0,
                                                  0, 0, 1, 0, 0, 0, 0, 1};
                    unsigned char tpx[8 * 8 * 4] = {};
                    OmWgpuClearColor blk6 = {0.0f, 0.0f, 0.0f, 1.0f};
                    if (!rt.clearAndDrawMeshTextured(blk6, identity6, th.view, mesh.vertexBuffer,
                                                     mesh.indexBuffer, mesh.indexCount, tpx)) {
                      OmLog::info("[OmWgpuRenderTarget] clearAndDrawMeshTextured returned false; "
                                  "probe=6 fails.");
                    } else {
                      dumpPpm(goldenDir, "probe6_mesh_textured", 8, 8, tpx);
                      const int idx6 = (4 * 8 + 4) * 4;
                      const unsigned r6 = tpx[idx6 + 0];
                      const unsigned g6 = tpx[idx6 + 1];
                      const unsigned b6 = tpx[idx6 + 2];
                      const unsigned a6 = tpx[idx6 + 3];
                      const bool ok6 = (r6 >= 200) && (g6 < 20) && (b6 < 20) && (a6 >= 0xfd);
                      OmLog::info(QString("[OmWgpuRenderTarget] clearAndDrawMeshTextured OK; "
                                          "pixel(4,4)=(%1,%2,%3,%4) expected (~255,~0,~0,~255) %5")
                                    .arg(r6)
                                    .arg(g6)
                                    .arg(b6)
                                    .arg(a6)
                                    .arg(ok6 ? "PASS" : "MISMATCH"));
                    }
                  }
                }
                if (probe >= 11) {
                  // R3.4-step-4 standalone probe: drive clearAndDrawScene
                  // with the same +Z-normal triangle from probe=4 to
                  // verify the scene pipeline itself works. Identity
                  // viewProj + identity model puts the triangle at
                  // NDC coords directly (-0.8..+0.8). The pixel center
                  // at (4,4) lands inside the triangle and should
                  // light up with `max(normal, 0) * intensity` =
                  // (0, 0, 1) * (0.25 + diff). With light (0.3, 0.4,
                  // -0.85), diff = dot(N=+Z, -L=(−0.3,−0.4,+0.85)) =
                  // 0.85, intensity = 1.0, so color = (0, 0, 1) = blue.
                  // Wait — that's the *normal* shader. kSolidLit uses
                  // baseColor instead. Let's expect baseColor (1, 0, 0)
                  // = red since we pass red here.
                  const float identityM[16] = {
                    1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
                  const float lightDA[4] = {0.3f, 0.4f, -0.85f, 0.25f};
                  OmWgpuSolidDraw d = {};
                  d.modelMatrix16 = identityM;
                  d.baseColorR = 1.0f;
                  d.baseColorG = 0.0f;
                  d.baseColorB = 0.0f;
                  d.baseColorA = 1.0f;
                  d.vertexBuffer = mesh.vertexBuffer;
                  d.indexBuffer = mesh.indexBuffer;
                  d.indexCount = mesh.indexCount;
                  unsigned char spx[8 * 8 * 4] = {};
                  OmWgpuClearColor blkS = {0.0f, 0.0f, 0.0f, 1.0f};
                  if (!rt.clearAndDrawScene(blkS, identityM, lightDA, &d, 1, spx)) {
                    OmLog::info("[OmWgpuRenderTarget] clearAndDrawScene single-draw returned "
                                "false; probe=11 fails.");
                  } else {
                    const int idxS = (4 * 8 + 4) * 4;
                    const unsigned rS = spx[idxS + 0];
                    const unsigned gS = spx[idxS + 1];
                    const unsigned bS = spx[idxS + 2];
                    const unsigned aS = spx[idxS + 3];
                    // Expect red (baseColor) * intensity ~ 1.0 = (~255, 0, 0, 255)
                    const bool okS = (rS >= 200) && (gS < 20) && (bS < 20) && (aS >= 0xfd);
                    OmLog::info(QString("[OmWgpuRenderTarget] clearAndDrawScene OK; "
                                        "pixel(4,4)=(%1,%2,%3,%4) expected ~(255,0,0,255) %5")
                                  .arg(rS)
                                  .arg(gS)
                                  .arg(bS)
                                  .arg(aS)
                                  .arg(okS ? "PASS" : "MISMATCH"));
                  }
                }
                if (probe >= 10) {
                  // R3.5b probe: build a tiny 2x2 solid-red QImage in
                  // ARGB32 (the format OmImageTexture loads into mImage),
                  // run it through the adapter, draw with the textured
                  // pipeline. Same expected output as probe=6 — center
                  // pixel should be ~pure red — but the upload path
                  // this time is QImage -> adapter -> RGBA8888 ->
                  // wgpuQueueWriteTexture instead of the hand-built
                  // byte array of probe=6.
                  QImage img(2, 2, QImage::Format_ARGB32);
                  img.fill(QColor(255, 0, 0, 255));
                  OmWgpuTextureCache tcache(vb);
                  OmWgpuTextureHandle th = OmWgpuImageAdapter::acquireFromQImage(
                      tcache, 0xC0DEC0DE, img);
                  if (!th.view) {
                    OmLog::info("[OmWgpuImageAdapter] acquireFromQImage failed; probe=10 stops.");
                  } else {
                    const float identityA[16] = {1, 0, 0, 0, 0, 1, 0, 0,
                                                  0, 0, 1, 0, 0, 0, 0, 1};
                    unsigned char qpx[8 * 8 * 4] = {};
                    OmWgpuClearColor blkA = {0.0f, 0.0f, 0.0f, 1.0f};
                    if (!rt.clearAndDrawMeshTextured(blkA, identityA, th.view, mesh.vertexBuffer,
                                                     mesh.indexBuffer, mesh.indexCount, qpx)) {
                      OmLog::info("[OmWgpuRenderTarget] clearAndDrawMeshTextured on adapter "
                                  "texture failed; probe=10 fails.");
                    } else {
                      dumpPpm(goldenDir, "probe10_qimage_adapter", 8, 8, qpx);
                      const int idxA = (4 * 8 + 4) * 4;
                      const unsigned ra = qpx[idxA + 0];
                      const unsigned ga = qpx[idxA + 1];
                      const unsigned ba = qpx[idxA + 2];
                      const unsigned aa = qpx[idxA + 3];
                      const bool okA = (ra >= 200) && (ga < 20) && (ba < 20) && (aa >= 0xfd);
                      OmLog::info(QString("[OmWgpuImageAdapter] acquireFromQImage+draw OK; "
                                          "pixel(4,4)=(%1,%2,%3,%4) expected ~(255,0,0,255) %5")
                                    .arg(ra)
                                    .arg(ga)
                                    .arg(ba)
                                    .arg(aa)
                                    .arg(okA ? "PASS" : "MISMATCH"));
                    }
                  }
                }
                if (probe >= 9) {
                  // R3.7b probe: smoke the Newton -> wgpu storage-buffer
                  // snapshot API. At main() startup there's no world +
                  // no Newton runtime, so the snapshot returns -1
                  // (correct, documented behavior). We just verify the
                  // symbol resolves + the early-out path fires. The
                  // happy-path runtime test waits on the R3.4-step-4
                  // Camera scene walk + a Newton-backed test world.
                  float xyzwBuf[4 * 8] = {};
                  OmPhysicsBackend *back = OmPhysicsBackendRegistry::newtonBackend();
                  OmNewtonBackend *nb = dynamic_cast<OmNewtonBackend *>(back);
                  if (!nb) {
                    OmLog::info("[OmNewtonBackend] snapshotBodyTranslations API: "
                                "newton backend symbol missing; probe=9 fails.");
                  } else {
                    const int n = nb->snapshotBodyTranslations(8, xyzwBuf);
                    const bool ok9 = (n == -1);  // expected: no runtime
                    OmLog::info(QString("[OmNewtonBackend] snapshotBodyTranslations(8) "
                                        "with no runtime returned %1 (expected -1) %2")
                                  .arg(n)
                                  .arg(ok9 ? "PASS" : "MISMATCH"));
                  }
                }
                if (probe >= 7) {
                  // R3.7 probe: draw 2 instances of a small (~0.3
                  // half-width) triangle from a storage buffer of
                  // 2 body translations (left + right). On an 8x8
                  // RTT in NDC space, x=-0.5 puts the triangle on
                  // the left half, x=+0.5 puts it on the right.
                  // We sample one pixel from each half:
                  //   pixel (2,4) should land inside the left
                  //   instance and pick up the blue normal color.
                  //   pixel (6,4) should land inside the right
                  //   instance and pick up the blue normal color.
                  // Mid pixels (3..4) hit the gap and stay black.
                  const float bodies[8] = {
                    -0.5f, 0.0f, 0.0f, 0.0f,
                     0.5f, 0.0f, 0.0f, 0.0f,
                  };
                  // Replace the mesh with a smaller triangle so the
                  // instances don't overlap. We re-acquire under a
                  // different mesh ID so the cache doesn't return
                  // the wide step-2 verts.
                  const float smallVerts[24] = {
                    -0.2f, -0.3f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                     0.2f, -0.3f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f,
                     0.0f,  0.3f, 0.0f, 0.0f, 0.0f, 1.0f, 0.5f, 1.0f,
                  };
                  OmWgpuMeshHandle small =
                      cache.acquire(0xC0DE7, smallVerts, sizeof(smallVerts), indices,
                                     sizeof(indices), 3, 32);
                  if (!small.vertexBuffer) {
                    OmLog::info("[OmWgpuMeshCache] small-mesh acquire failed; probe=7 stops.");
                  } else {
                    const float identity7[16] = {1, 0, 0, 0, 0, 1, 0, 0,
                                                  0, 0, 1, 0, 0, 0, 0, 1};
                    unsigned char ipx[8 * 8 * 4] = {};
                    OmWgpuClearColor blk7 = {0.0f, 0.0f, 0.0f, 1.0f};
                    if (!rt.clearAndDrawInstanced(blk7, identity7, bodies, 2,
                                                   small.vertexBuffer, small.indexBuffer,
                                                   small.indexCount, ipx)) {
                      OmLog::info("[OmWgpuRenderTarget] clearAndDrawInstanced returned false; "
                                  "probe=7 fails.");
                    } else {
                      dumpPpm(goldenDir, "probe7_instanced_2bodies", 8, 8, ipx);
                      const int idxL = (4 * 8 + 2) * 4;
                      const int idxR = (4 * 8 + 6) * 4;
                      const unsigned bl = ipx[idxL + 2];
                      const unsigned br = ipx[idxR + 2];
                      const bool ok7 = (bl >= 200) && (br >= 200);
                      OmLog::info(QString("[OmWgpuRenderTarget] clearAndDrawInstanced OK; "
                                          "pixel(2,4).b=%1 pixel(6,4).b=%2 "
                                          "both expected ~255 (storage-buffer instancing) %3")
                                    .arg(bl)
                                    .arg(br)
                                    .arg(ok7 ? "PASS" : "MISMATCH"));
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }

#ifdef _WIN32
  const QString MSYS2_HOME = QDir::fromNativeSeparators(getenv("MSYS2_HOME"));
  if (MSYS2_HOME.isEmpty())                                              // Webots was not started from a MSYS2 console
    qputenv("MSYS2_HOME", QString(omnisimDirPath + "/msys64").toUtf8());  // useful to Python >= 3.8 controllers
  const QString relativeQtPluginsPath("/mingw64/share/qt6/plugins");
  const QString bundledQtPluginsPath(omnisimDirPath + "/msys64" + relativeQtPluginsPath);
  const QString qtPluginsPath = QDir(bundledQtPluginsPath).exists() ? bundledQtPluginsPath : MSYS2_HOME + relativeQtPluginsPath;
#endif

  const QString QT_QPA_PLATFORM_PLUGIN_PATH = qEnvironmentVariable("QT_QPA_PLATFORM_PLUGIN_PATH");
  if (QT_QPA_PLATFORM_PLUGIN_PATH.isEmpty())
    QCoreApplication::addLibraryPath(
#ifdef _WIN32
      qtPluginsPath
#elif defined(__APPLE__)
      omnisimDirPath + "/Contents/lib/webots/qt/plugins"
#else
      omnisimDirPath + "/lib/webots/qt/plugins"
#endif
    );

#ifdef __APPLE__
  QString qtFiltersFilePath = QDir::fromNativeSeparators(omnisimDirPath + "/Contents/Resources/qt_warning_filters.conf");
#else
  QString qtFiltersFilePath = QDir::fromNativeSeparators(omnisimDirPath + "/resources/qt_warning_filters.conf");
#endif
  // load qt warning filters from file
  QFile qtFiltersFile(qtFiltersFilePath);
  if (qtFiltersFile.open(QIODevice::ReadOnly)) {
    QTextStream in(&qtFiltersFile);
    QString line;
    while (!in.atEnd()) {
      line = in.readLine();
      line = line.trimmed();
      if (line.startsWith("#") || line.isEmpty())
        continue;
      QRegularExpression *re = new QRegularExpression(
        line, QRegularExpression::ExtendedPatternSyntaxOption | QRegularExpression::UseUnicodePropertiesOption);
      if (re->isValid())
        gQtMessageFilters.append(re);
      else {
        QString message = QString("regular expression '%1' in file '%2' is invalid: %3")
                            .arg(line)
                            .arg(qtFiltersFilePath)
                            .arg(re->errorString());
        fprintf(stderr, "%s\n", message.toUtf8().constData());
      }
    }
  } else {
    QString message = QString("File not found: '%1'.").arg(qtFiltersFilePath);
    fprintf(stderr, "%s\n", message.toUtf8().constData());
  }

  // Putting break points in the catchMessageOutput and getting the stack allows to determine
  // efficiently what OmniSim statement is responsible to generate some Qt output
  qInstallMessageHandler(catchMessageOutput);

  QApplication::setAttribute(Qt::AA_Use96Dpi);

#ifdef _WIN32
  // fixes truncated menus on some screen configurations: https://bugreports.qt.io/browse/QTBUG-98347
  QGuiApplication::setHighDpiScaleFactorRoundingPolicy(Qt::HighDpiScaleFactorRoundingPolicy::Floor);
#endif

  // Compute-only headless mode (OMNISIM_NO_GL, core-evolution-plan.md Phase Q1 Tier C):
  // no window and no GL context will ever be created, so default the Qt platform to
  // "minimal" -- no window-system connection at all. On Linux this removes the
  // X/Wayland/Xvfb requirement for compute-only runs entirely. Decided here because the
  // platform is locked in when the QApplication below constructs; an explicit
  // QT_QPA_PLATFORM from the caller always wins.
  if (qEnvironmentVariableIsSet("OMNISIM_NO_GL") && !qEnvironmentVariableIsSet("QT_QPA_PLATFORM"))
    qputenv("QT_QPA_PLATFORM", "minimal");

  OmGuiApplication app(argc, argv);
  // Quit the application correctly when receiving POSIX signals.
  signal(SIGINT, quitApplication);  // this signal is working on Windows when Ctrl+C from cmd.exe.
#ifndef _WIN32
  signal(SIGTERM, quitApplication);
  signal(SIGQUIT, quitApplication);
  signal(SIGHUP, quitApplication);
  // Symptom: locale is wrong in dynamic libraries (i.e. a loaded plugin)
  // From http://qt-project.org/doc/qt-4.8/qcoreapplication.html :
  //   On Unix/Linux Qt is configured to use the system locale settings by default.
  //   This can cause a conflict when using POSIX functions, for instance, when
  //   converting between data types such as floats and strings, since the
  //   notation may differ between locales. To get around this problem, call
  //   the POSIX function setlocale(LC_NUMERIC,"C") right after initializing
  //   QApplication or QCoreApplication to reset the locale that is used for
  //   number formatting to "C"-locale.
  setlocale(LC_NUMERIC, "C");
#ifdef __APPLE__
  // 'LANG' can be set in the terminal.
  // - "fr_CH.UTF-8" may cause issues in procedural PROTO nodes.
  // - If "UTF-8" is not set, UTF-8 characters are not handled properly in some libraries, such as FreeType.
  setenv("LANG", "UTF-8", true);
#endif
#endif

  int result = app.exec();
  // Say how it ended, in the LOG, before closing it.
  //
  // A non-zero exit from a GUI-subsystem binary is otherwise completely mute:
  // there is no console for cerr, and a caller sees only "the process returned
  // 1 and produced nothing". Measured repeatedly on 2026-08-02 -- roughly one
  // cold headless launch in three ends this way, after the world parses and
  // before the physics backend initialises, leaving a log that stops mid-load
  // with no explanation. That failure is not diagnosed yet; this line at least
  // separates "the engine decided to stop" from "the engine was killed", which
  // a batch runner currently cannot tell apart and scores as a failure of
  // whatever it was measuring.
  if (result != 0)
    OmLog::error(QString("omnisim exited with code %1 -- the simulation did not run to completion. If the log above "
                         "ends mid-load with no error, this is the known intermittent cold-launch failure; re-run.")
                   .arg(result),
                 false, OmLog::ODE);
  else
    OmLog::info(QString("omnisim exited normally (code 0)"));
  OmLog::closeFileLog();
  return result;
}
