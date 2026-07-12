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

#include "WbWrenWindow.hpp"

#include "CpuPassTimer.hpp"
#include "GpuPassTimer.hpp"
#include "WbDragSolidEvent.hpp"
#include "WbLensFlare.hpp"
#include "WbLightRepresentation.hpp"
#include "WbLog.hpp"
#include "WbMessageBox.hpp"
#include "WbMultimediaStreamingServer.hpp"
#include "WbPerformanceLog.hpp"
#include "WbPreferences.hpp"
#include "WbSysInfo.hpp"
#include "WbVideoRecorder.hpp"
#include "WbViewpoint.hpp"
#include "WbWorld.hpp"
#include "WbWrenBloom.hpp"
#include "WbWrenLabelOverlay.hpp"
#include "WbWrenOpenGlContext.hpp"
#include "WbWrenPicker.hpp"
#include "WbWrenPostProcessingEffects.hpp"
#include "WbWrenShaders.hpp"
#include "WbWrenTextureOverlay.hpp"

#include <wren/config.h>
#include <wren/frame_buffer.h>
#include <wren/gl_state.h>
#include <wren/scene.h>
#include <wren/texture_rtt.h>
#include <wren/viewport.h>

#include <QtWidgets/QApplication>

#include <cassert>

WbWrenWindow *WbWrenWindow::cInstance = NULL;

const int PBO_COUNT = 2;

WbWrenWindow *WbWrenWindow::instance() {
  assert(cInstance);
  return cInstance;
}

WbWrenWindow::WbWrenWindow() :
  QWindow(),
  mUpdatePending(false),
  mSnapshotBuffer(NULL),
  mSnapshotBufferWidth(0),
  mSnapshotBufferHeight(0),
  mVideoPBOIndex(-1),
  mWrenMainFrameBuffer(NULL),
  mWrenMainFrameBufferTexture(NULL),
  mWrenNormalFrameBufferTexture(NULL),
  mWrenDepthFrameBufferTexture(NULL),
  mVideoStreamingServer(NULL) {
  assert(WbWrenWindow::cInstance == NULL);
  WbWrenWindow::cInstance = this;

  setSurfaceType(QWindow::OpenGLSurface);

  const WbVersion openGLTargetVersion(3, 3);

  QSurfaceFormat format = requestedFormat();
  format.setVersion(openGLTargetVersion.majorNumber(), openGLTargetVersion.minorNumber());
  format.setProfile(QSurfaceFormat::CoreProfile);
  format.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
  format.setRedBufferSize(8);
  format.setGreenBufferSize(8);
  format.setBlueBufferSize(8);
#ifndef __APPLE__  // specifying alpha buffer size on macOS causes flickering when resizing the viewport
  format.setAlphaBufferSize(8);
#endif
  format.setDepthBufferSize(24);
  format.setStencilBufferSize(8);
  format.setSwapInterval(0);
  setFormat(format);

  WbWrenOpenGlContext::init(this, this, format);

  if (!WbWrenOpenGlContext::instance()->isValid())
    WbLog::fatal(tr("OmniSim could not initialize the rendering system.\n"
                    "Please check your GPU abilities and install the latest graphics drivers.\n"
                    "Please do also check that your graphics hardware meets the requirements specified in the User Guide."));

  const WbVersion openGLActualVersion(WbWrenOpenGlContext::instance()->format().majorVersion(),
                                      WbWrenOpenGlContext::instance()->format().minorVersion());

  if (openGLActualVersion < openGLTargetVersion)
    WbLog::fatal(tr("OmniSim requires OpenGL %1 while only OpenGL %2 can be initialized.\n"
                    "Please check your GPU abilities and install the latest graphics drivers.\n"
                    "Please do also check that your graphics hardware meets the requirements specified in the User Guide.")
                   .arg(openGLTargetVersion.toString(false))
                   .arg(openGLActualVersion.toString(false)));
}

WbWrenWindow::~WbWrenWindow() {
  WbWrenOpenGlContext::makeWrenCurrent();
  WbWrenPostProcessingEffects::clearResources();

  if (mWrenMainFrameBuffer)
    wr_frame_buffer_delete(mWrenMainFrameBuffer);

  if (mWrenMainFrameBufferTexture)
    wr_texture_delete(WR_TEXTURE(mWrenMainFrameBufferTexture));

  if (mWrenNormalFrameBufferTexture)
    wr_texture_delete(WR_TEXTURE(mWrenNormalFrameBufferTexture));

  if (mWrenDepthFrameBufferTexture)
    wr_texture_delete(WR_TEXTURE(mWrenDepthFrameBufferTexture));

  mWrenMainFrameBuffer = NULL;
  mWrenMainFrameBufferTexture = NULL;
  mWrenNormalFrameBufferTexture = NULL;
  mWrenDepthFrameBufferTexture = NULL;

  wr_scene_destroy();

  // delete shaders on exit
  WbWrenShaders::deleteShaders();

#ifndef _WIN32
  destroy();
#endif

  WbWrenOpenGlContext::doneWren();
  WbWrenOpenGlContext::destroy();

  delete[] mSnapshotBuffer;
}

// A custom initialization function is used here,
// because using the real one implies some issues on Mac since Qt 5.2.1
// Calling winId() creates the actual window and passes it 2 times into QGLWidget
// Using a custom initialization allows to make sure OpenGL is initialized after the
// QWidget creation (so the winId() is correct)
void WbWrenWindow::initialize() {
  if (wr_gl_state_is_initialized())
    return;

  // Moving these calls into the constructor causes rendering issues on Windows
  create();

  WbWrenOpenGlContext::makeWrenCurrent();

  wr_scene_init(wr_scene_get_instance());

  // Useful for debugging
  // wr_config_set_bounding_volume_program(WbWrenShaders::boundingVolumeShader());
  // wr_config_set_show_axis_aligned_bounding_boxes(true);
  // wr_config_set_show_shadow_axis_aligned_bounding_boxes(true);
  // wr_config_set_show_bounding_spheres(true);

  // Workaround an OpenGL driver bug occuring in VMWare virtual machines:
  // - The OpenGL state when calling the glDrawElements function may be corrupted.
  //   In such case, the previous vertex buffers may be overriden with the current material.
  wr_config_set_requires_flush_after_draw(WbSysInfo::isVirtualMachine());

  // Workaround an OpenGL driver bug occuring in VMWare virtual machines:
  // - The OpenGL depth buffer returns the square root of the expected value when getting the depth buffer.
  wr_config_set_requires_depth_buffer_distortion(WbSysInfo::isVirtualMachine());

  updateFrameBuffer();

  wr_scene_set_fog_program(wr_scene_get_instance(), WbWrenShaders::fogShader());
  wr_scene_set_shadow_volume_program(wr_scene_get_instance(), WbWrenShaders::shadowVolumeShader());

  WbWrenOpenGlContext::doneWren();
  WbWrenPostProcessingEffects::loadResources();
  updateWrenViewportDimensions();
}

void WbWrenWindow::updateWrenViewportDimensions() {
  const int ratio = (int)devicePixelRatio();
  wr_viewport_set_pixel_ratio(wr_scene_get_viewport(wr_scene_get_instance()), ratio);
  WbVideoRecorder::instance()->setScreenPixelRatio(ratio);
}

void WbWrenWindow::blitMainFrameBufferToScreen() {
  wr_frame_buffer_blit_to_screen(mWrenMainFrameBuffer);
}

void WbWrenWindow::renderLater() {
  if (!mUpdatePending) {
    mUpdatePending = true;
    QApplication::postEvent(this, new QEvent(QEvent::UpdateRequest));
  }
}

void WbWrenWindow::renderNow(bool culling, bool offScreen) {
  if ((!isExposed() && !offScreen) || !wr_gl_state_is_initialized())
    return;

  static int first = true;
#ifdef __APPLE__
  // Make sure all events are processed before first render, omitting this snippet
  // causes graphical corruption on macOS due to the main framebuffer being invalid.
  // On Windows, this fix causes a crash on startup for certain worlds.
  if (first)
    QCoreApplication::processEvents(QEventLoop::AllEvents);
#endif

  // T2.1 — wall-clock CPU timer covering the entire renderNow body
  // (everything WbViewpoint touches plus the wr_scene_render call itself
  // plus swapBuffers' vsync wait). No-op unless OMNISIM_RENDERER_TIMINGS
  // is set. Pairs with the GPU "SceneRender" timer below — the gap
  // between the two is webots-side overhead + driver / vsync wait.
  static CpuPassTimer fullFrameTimer("FullRenderNow");
  fullFrameTimer.begin();

  WbPerformanceLog *log = WbPerformanceLog::instance();
  if (log)
    log->startMeasure(WbPerformanceLog::MAIN_RENDERING);

  WbViewpoint *viewpoint = WbWorld::instance() ? WbWorld::instance()->viewpoint() : NULL;
  if (viewpoint) {
    viewpoint->enableNodeVisibility(false);
    viewpoint->updatePostProcessingParameters();
  }

  WbWrenOpenGlContext::makeWrenCurrent();

  // GPU-side timer for just the scene render call.
  static GpuPassTimer sceneTimer("SceneRender");

  // T2.1.c — when timing is enabled, ask WREN to break SceneRender down
  // into forward render vs post-processing. WREN's enable call happens
  // exactly once on first frame; after that we just read the latest
  // available numbers via the getter.
  static bool wrenPassTimingRequested = false;
  if (GpuPassTimer::isEnabled() && !wrenPassTimingRequested) {
    wr_scene_set_pass_timing_enabled(true);
    wr_scene_set_render_counts_enabled(true);
    wrenPassTimingRequested = true;
  }

  sceneTimer.begin();
  wr_scene_render(wr_scene_get_instance(), NULL, culling, offScreen);
  sceneTimer.end();
  sceneTimer.poll();

  // Log WREN-internal forward / post-process breakdown alongside the
  // SceneRender total. Same cadence as the other timers (~2 s).
  if (GpuPassTimer::isEnabled()) {
    static double forwardSumMs = 0.0;
    static double postSumMs = 0.0;
    static double ambientSumMs = 0.0;
    static double perLightSumMs = 0.0;
    static double residualSumMs = 0.0;
    static int sampleCount = 0;
    static int framesSinceLog = 0;
    unsigned long long forwardNs = 0;
    unsigned long long postNs = 0;
    unsigned long long ambientNs = 0;
    unsigned long long perLightNs = 0;
    unsigned long long residualNs = 0;
    wr_scene_get_last_pass_timings_ns(&forwardNs, &postNs);
    wr_scene_get_last_forward_subpass_timings_ns(&ambientNs, &perLightNs, &residualNs);
    if (forwardNs > 0 || postNs > 0) {
      forwardSumMs += static_cast<double>(forwardNs) / 1.0e6;
      postSumMs += static_cast<double>(postNs) / 1.0e6;
      ambientSumMs += static_cast<double>(ambientNs) / 1.0e6;
      perLightSumMs += static_cast<double>(perLightNs) / 1.0e6;
      residualSumMs += static_cast<double>(residualNs) / 1.0e6;
      ++sampleCount;
    }
    ++framesSinceLog;
    if (framesSinceLog >= 120 && sampleCount > 0) {
      const double avgForward = forwardSumMs / sampleCount;
      const double avgPost = postSumMs / sampleCount;
      const double avgAmbient = ambientSumMs / sampleCount;
      const double avgPerLight = perLightSumMs / sampleCount;
      const double avgResidual = residualSumMs / sampleCount;
      WbLog::info(
        QCoreApplication::translate("WbWrenWindow",
          "GPU breakdown: forward = %1 ms (ambient %2 + perLight %3 + residual %4), post-process = %5 ms (avg of %6 samples)")
          .arg(avgForward, 0, 'f', 3)
          .arg(avgAmbient, 0, 'f', 3)
          .arg(avgPerLight, 0, 'f', 3)
          .arg(avgResidual, 0, 'f', 3)
          .arg(avgPost, 0, 'f', 3)
          .arg(sampleCount));

      // All-viewports aggregate — main window plus every camera / lidar /
      // range-finder sensor. On a husky-heavy world the sensors are the
      // dominant rendering load and the single-viewport breakdown above
      // hides them. Divide totals by ~120 main frames to get per-main-
      // frame cost. harvestedCount may be < callCount when the 4-slot
      // GPU query ring laps under heavy sensor traffic.
      unsigned long long aggForwardNs = 0, aggPostNs = 0;
      unsigned int aggCallCount = 0, aggHarvestedCount = 0;
      wr_scene_get_aggregate_render_timings_ns(&aggForwardNs, &aggPostNs, &aggCallCount, &aggHarvestedCount);
      const double aggForwardMs = static_cast<double>(aggForwardNs) / 1.0e6;
      const double aggPostMs = static_cast<double>(aggPostNs) / 1.0e6;
      const double mainFrames = static_cast<double>(framesSinceLog);
      const double aggForwardPerMainFrameMs = mainFrames > 0 ? aggForwardMs / mainFrames : 0.0;
      const double aggPostPerMainFrameMs = mainFrames > 0 ? aggPostMs / mainFrames : 0.0;
      const double callsPerMainFrame =
        mainFrames > 0 ? static_cast<double>(aggCallCount) / mainFrames : 0.0;
      WbLog::info(
        QCoreApplication::translate("WbWrenWindow",
          "  all viewports: %1 calls/frame (%2 harvested), forward+post = %3+%4 ms/frame")
          .arg(callsPerMainFrame, 0, 'f', 2)
          .arg(aggHarvestedCount)
          .arg(aggForwardPerMainFrameMs, 0, 'f', 3)
          .arg(aggPostPerMainFrameMs, 0, 'f', 3));
      wr_scene_reset_aggregate_render_timings();

      // Render-count snapshot of the most recent frame. Numbers are
      // per-frame integers, not averaged — they reflect scene size, which
      // changes on world load / robot spawn rather than continuously.
      int enq = 0, vis = 0, drawn = 0;
      wr_scene_get_last_render_counts(&enq, &vis, &drawn);
      const long long tris = wr_scene_get_last_triangles_drawn();
      WbLog::info(
        QCoreApplication::translate("WbWrenWindow",
          "Renderables: enqueued = %1, visible = %2, drawn = %3 (post frustum cull); triangles = %4")
          .arg(enq)
          .arg(vis)
          .arg(drawn)
          .arg(tris));

      // Top-mesh histogram — instancing candidates. A mesh that appears
      // many times tells us how big the win is from instancing it.
      WrSceneMeshHistogramEntry hist[8];
      int histFilled = 0;
      wr_scene_get_top_mesh_histogram(hist, 8, &histFilled);
      for (int i = 0; i < histFilled; ++i) {
        WbLog::info(
          QCoreApplication::translate("WbWrenWindow",
            "  mesh #%1 id=%2 drawn=%3 vtx=%4 tri=%5")
            .arg(i + 1)
            .arg(hist[i].mesh_id_low32, 8, 16, QChar('0'))
            .arg(hist[i].draw_count)
            .arg(hist[i].vertex_count)
            .arg(hist[i].triangle_count));
      }

      forwardSumMs = 0.0;
      postSumMs = 0.0;
      ambientSumMs = 0.0;
      perLightSumMs = 0.0;
      residualSumMs = 0.0;
      sampleCount = 0;
      framesSinceLog = 0;
    }
  }

  if (!offScreen)
    WbWrenOpenGlContext::instance()->swapBuffers(this);
  WbWrenOpenGlContext::doneWren();

  if (mVideoStreamingServer && mVideoStreamingServer->isNewFrameNeeded() && !first)
    // Skip the first call to 'renderNow()' because OpenGL context seems to be not ready. Not skipping causes a freeze.
    mVideoStreamingServer->sendImage(grabWindowBufferNow());

  if (log)
    log->stopMeasure(WbPerformanceLog::MAIN_RENDERING);

  fullFrameTimer.end();
  fullFrameTimer.poll();

  if (first)
    first = false;
}

bool WbWrenWindow::event(QEvent *event) {
  switch (event->type()) {
    case QEvent::UpdateRequest:
      if (mUpdatePending) {
        mUpdatePending = false;
        renderNow();
      }
      return true;
    case QEvent::Expose:
    case QEvent::Move:
    case QEvent::Resize:
      resizeWren(width(), height());
      return true;
    default:
      return QWindow::event(event);
  }
}

void WbWrenWindow::resizeWren(int width, int height) {
  if (!wr_gl_state_is_initialized())
    return;

  WbWrenOpenGlContext::makeWrenCurrent();

  updateFrameBuffer();

  WbWrenTextureOverlay::updateOverlayDimensions();
  WbWrenLabelOverlay::updateOverlaysDimensions();
  WbLightRepresentation::updateScreenScale(width, height);

  if (WbWorld::instance() && WbWorld::instance()->viewpoint())
    WbWorld::instance()->viewpoint()->updatePostProcessingEffects();

  WbWrenOpenGlContext::doneWren();

  renderLater();

  emit resized();
}

void WbWrenWindow::flipAndScaleDownImageBuffer(const unsigned char *source, unsigned char *destination, int sourceWidth,
                                               int sourceHeight, int scaleDownFactor) {
  // flip vertically the image and scale it down (about 3x faster than QImage::mirrored(), QImage::scaled())
  // cppcheck-suppress unsafeClassDivZero
  const int h = sourceHeight / scaleDownFactor;
  const int w = sourceWidth / scaleDownFactor;
  const int yFactor = scaleDownFactor * sourceWidth;

  // - The `unsigned char *` to `int *` cast is possible assuming that a pixel is coded as four bytes (RGBA)
  //   aligned on an `int *` boundary.
  // - A preliminary `unsigned char *` to `void *` cast is required to by-pass "cast-align" clang warnings.
  const uint32_t *src = static_cast<const uint32_t *>(static_cast<void *>(const_cast<unsigned char *>(source)));
  uint32_t *dst = static_cast<uint32_t *>(static_cast<void *>(destination));

  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++)
      dst[(h - 1 - y) * w + x] = src[y * yFactor + x * scaleDownFactor];
  }
}

QImage WbWrenWindow::grabWindowBufferNow() {
  WbWrenOpenGlContext::makeWrenCurrent();

  const int destinationWidth = width();
  const int destinationHeight = height();
  if (mSnapshotBuffer == NULL || destinationWidth != mSnapshotBufferWidth || destinationHeight != mSnapshotBufferHeight) {
    delete[] mSnapshotBuffer;
    mSnapshotBufferWidth = destinationWidth;
    mSnapshotBufferHeight = destinationHeight;
    mSnapshotBuffer = new unsigned char[4 * destinationWidth * destinationHeight];
  }
  const int sourceWidth = destinationWidth;
  const int sourceHeight = destinationHeight;
  unsigned char *temp = new unsigned char[4 * sourceWidth * sourceHeight];
  readPixels(sourceWidth, sourceHeight, GL_BGRA, temp);
  flipAndScaleDownImageBuffer(temp, mSnapshotBuffer, sourceWidth, sourceHeight, 1.0);
  delete[] temp;
  WbWrenOpenGlContext::doneWren();

  return QImage(mSnapshotBuffer, mSnapshotBufferWidth, mSnapshotBufferHeight, QImage::Format_RGB32);
}

void WbWrenWindow::initVideoPBO() {
  WbWrenOpenGlContext::makeWrenCurrent();

  mVideoWidth = width();
  mVideoHeight = height();
  const int size = 4 * mVideoWidth * mVideoHeight;
  wr_scene_init_frame_capture(wr_scene_get_instance(), PBO_COUNT, mVideoPBOIds, size);
  mVideoPBOIndex = -1;

  WbWrenOpenGlContext::doneWren();
}

void WbWrenWindow::completeVideoPBOProcessing(bool canceled) {
  WbWrenOpenGlContext::makeWrenCurrent();

  // process last frame
  if (!canceled)
    processVideoPBO();
  mVideoPBOIndex = -1;
  wr_scene_terminate_frame_capture(wr_scene_get_instance());

  WbWrenOpenGlContext::doneWren();
}

void WbWrenWindow::processVideoPBO() {
  if (mVideoPBOIndex < 0)
    return;

  WbWrenOpenGlContext::makeWrenCurrent();

  // Process previously copied pixels
  WrScene *scene = wr_scene_get_instance();
  wr_scene_bind_pixel_buffer(scene, mVideoPBOIds[mVideoPBOIndex]);
  unsigned char *buffer = static_cast<unsigned char *>(wr_scene_map_pixel_buffer(scene, GL_READ_ONLY));
  if (buffer) {
    emit videoImageReady(buffer);
    wr_scene_unmap_pixel_buffer(scene);
  }

  WbWrenOpenGlContext::doneWren();
}

void WbWrenWindow::updateFrameBuffer() {
  recreateMainFrameBuffer(width(), height());
}

// updateFrameBuffer's body, parametrized on the target size so grabSceneOffscreen can render the
// scene at a FIXED resolution independent of the window/layout (the deterministic parity golden).
void WbWrenWindow::recreateMainFrameBuffer(int w, int h) {
  WbWrenOpenGlContext::makeWrenCurrent();

  if (mWrenMainFrameBuffer)
    wr_frame_buffer_delete(mWrenMainFrameBuffer);

  if (mWrenMainFrameBufferTexture)
    wr_texture_delete(WR_TEXTURE(mWrenMainFrameBufferTexture));

  if (mWrenNormalFrameBufferTexture)
    wr_texture_delete(WR_TEXTURE(mWrenNormalFrameBufferTexture));

  if (mWrenDepthFrameBufferTexture)
    wr_texture_delete(WR_TEXTURE(mWrenDepthFrameBufferTexture));

  mWrenMainFrameBuffer = wr_frame_buffer_new();
  wr_frame_buffer_set_size(mWrenMainFrameBuffer, w, h);

  mWrenMainFrameBufferTexture = wr_texture_rtt_new();
  wr_texture_set_internal_format(WR_TEXTURE(mWrenMainFrameBufferTexture), WR_TEXTURE_INTERNAL_FORMAT_RGB16F);

  mWrenNormalFrameBufferTexture = wr_texture_rtt_new();
  wr_texture_set_internal_format(WR_TEXTURE(mWrenNormalFrameBufferTexture), WR_TEXTURE_INTERNAL_FORMAT_RGB8);

  wr_frame_buffer_append_output_texture(mWrenMainFrameBuffer, mWrenMainFrameBufferTexture);
  wr_frame_buffer_append_output_texture(mWrenMainFrameBuffer, mWrenNormalFrameBufferTexture);
  wr_frame_buffer_enable_depth_buffer(mWrenMainFrameBuffer, true);

  mWrenDepthFrameBufferTexture = wr_texture_rtt_new();
  wr_texture_set_internal_format(WR_TEXTURE(mWrenDepthFrameBufferTexture), WR_TEXTURE_INTERNAL_FORMAT_DEPTH24_STENCIL8);
  wr_frame_buffer_set_depth_texture(mWrenMainFrameBuffer, mWrenDepthFrameBufferTexture);

  wr_frame_buffer_setup(mWrenMainFrameBuffer);
  wr_viewport_set_frame_buffer(wr_scene_get_viewport(wr_scene_get_instance()), mWrenMainFrameBuffer);

  wr_viewport_set_size(wr_scene_get_viewport(wr_scene_get_instance()), w, h);

  WbWrenOpenGlContext::doneWren();
}

QImage WbWrenWindow::grabSceneOffscreen(int w, int h) {
  if (w <= 0 || h <= 0 || !wr_gl_state_is_initialized())
    return QImage();
  // Render the live scene into a w×h main framebuffer (offScreen=true: no exposure requirement, no
  // swap), read it back, then restore the window-sized buffer and queue a normal repaint.
  // CRITICAL: after each framebuffer recreation the viewpoint's post-processing effects (GTAO,
  // SMAA, bloom, ...) must be rewired to the NEW buffer — exactly what resizeWren does. Without it
  // they sample/render the DELETED framebuffer and the GUI crashes inside the offscreen render
  // (the bug that originally made this grab unusable).
  recreateMainFrameBuffer(w, h);
  WbWrenOpenGlContext::makeWrenCurrent();
  if (WbWorld::instance() && WbWorld::instance()->viewpoint())
    WbWorld::instance()->viewpoint()->updatePostProcessingEffects();
  WbWrenOpenGlContext::doneWren();
  renderNow(true, /*offScreen=*/true);
  WbWrenOpenGlContext::makeWrenCurrent();
  unsigned char *tmp = new unsigned char[4 * w * h];
  readPixels(w, h, GL_BGRA, tmp);
  QImage out(w, h, QImage::Format_RGB32);
  flipAndScaleDownImageBuffer(tmp, out.bits(), w, h, 1);
  delete[] tmp;
  WbWrenOpenGlContext::doneWren();
  recreateMainFrameBuffer(width(), height());
  WbWrenOpenGlContext::makeWrenCurrent();
  if (WbWorld::instance() && WbWorld::instance()->viewpoint())
    WbWorld::instance()->viewpoint()->updatePostProcessingEffects();
  WbWrenOpenGlContext::doneWren();
  renderLater();
  return out;
}

void WbWrenWindow::requestGrabWindowBuffer() {
  WbWrenOpenGlContext::makeWrenCurrent();

  // Asynchronous pixels copy from GPU
  if (mVideoPBOIndex >= 0)
    // process previous frame image stored in PBO
    processVideoPBO();

  WrScene *scene = wr_scene_get_instance();

  mVideoPBOIndex = (mVideoPBOIndex + 1) % PBO_COUNT;
  // Request pixels copy
  // read pixels from framebuffer to PBO: wr_scene_get_main_buffer() should return immediately
  wr_scene_bind_pixel_buffer(scene, mVideoPBOIds[mVideoPBOIndex]);
  readPixels(mVideoWidth, mVideoHeight, GL_BGRA, 0);
  wr_scene_bind_pixel_buffer(scene, 0);

  WbWrenOpenGlContext::doneWren();
}

QSize WbWrenWindow::minimumSize() const {
  return QSize(1, 1);
}

QSize WbWrenWindow::sizeHint() const {
  return QSize(400, 400);
}

void WbWrenWindow::setVideoStreamingServer(WbMultimediaStreamingServer *streamingServer) {
  mVideoStreamingServer = streamingServer;
  connect(mVideoStreamingServer, &WbMultimediaStreamingServer::imageRequested, this, &WbWrenWindow::feedMultimediaStreamer);
}

void WbWrenWindow::feedMultimediaStreamer() {
  renderNow();
}

void WbWrenWindow::readPixels(int width, int height, unsigned int format, void *buffer) {
  wr_scene_get_main_buffer(width, height, format, GL_UNSIGNED_BYTE, buffer);
}
