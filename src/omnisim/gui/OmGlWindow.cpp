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

#include "OmGlWindow.hpp"

#include "OmLog.hpp"
#include "OmMultimediaStreamingServer.hpp"
#include "OmVersion.hpp"
#include "OmVideoRecorder.hpp"
#include "OmWrenOpenGlContext.hpp"

#include <QtWidgets/QApplication>

#include <cassert>

OmGlWindow *OmGlWindow::cInstance = NULL;

// D1.5 (WREN deleted at D1.4): the GL degrade is the SHIPPED path, unconditionally -- the
// scene renderer is wgpu and needs no GL, so a host whose OpenGL context cannot be created
// (or is below 3.3) loses only the GL present/blit fallback, device pop-out windows and the
// GPU pass timers, all of which gate on OmWrenOpenGlContext::isInitialized() (the same
// "no GL context exists" contract OMNISIM_NO_GL established). The OMNISIM_GL_OPTIONAL env
// conditioning is retired: the old fatals killed sessions that wgpu could serve fine.

OmGlWindow *OmGlWindow::instance() {
  assert(cInstance);
  return cInstance;
}

OmGlWindow::OmGlWindow() :
  QWindow(),
  mUpdatePending(false),
  mVideoStreamingServer(NULL) {
  assert(OmGlWindow::cInstance == NULL);
  OmGlWindow::cInstance = this;

  // TODO(R8, brief sect. 4): on a GL-less host it is unanswered whether an OpenGLSurface-typed
  // QWindow still yields a usable platform window — the surface that actually presents on the
  // wgpu path is the Vulkan child surface (OmView3D.cpp:2153-2157), not this one.
  setSurfaceType(QWindow::OpenGLSurface);

  const OmVersion openGLTargetVersion(3, 3);

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

  OmWrenOpenGlContext::init(this, this, format);

  if (!OmWrenOpenGlContext::instance()->isValid()) {
    OmWrenOpenGlContext::destroy();
    OmLog::warning(tr("OpenGL context creation FAILED -- continuing without GL (the scene renderer is wgpu). "
                      "The GL present/blit fallback, device pop-out windows and GPU pass timers are disabled; "
                      "wgpu presentation-free rendering and compute paths are unaffected."));
    return;
  }

  const OmVersion openGLActualVersion(OmWrenOpenGlContext::instance()->format().majorVersion(),
                                      OmWrenOpenGlContext::instance()->format().minorVersion());

  if (openGLActualVersion < openGLTargetVersion) {
    OmWrenOpenGlContext::destroy();
    OmLog::warning(tr("Only OpenGL %1 could be initialized (%2 wanted) -- continuing "
                      "without GL (the scene renderer is wgpu). GL present/blit, device pop-out windows and GPU pass timers "
                      "are disabled; wgpu presentation-free rendering and compute paths are unaffected.")
                     .arg(openGLActualVersion.toString(false))
                     .arg(openGLTargetVersion.toString(false)));
    return;
  }
}

OmGlWindow::~OmGlWindow() {
  // cInstance is deliberately NOT nulled here — the pre-split OmWrenWindow dtor never nulled
  // it either (behaviour parity; a second construction would trip the ctor assert, which no
  // code path exercises today).
#ifndef _WIN32
  destroy();
#endif

  OmWrenOpenGlContext::doneWren();
  OmWrenOpenGlContext::destroy();
}

// A custom initialization function is used here,
// because using the real one implies some issues on Mac since Qt 5.2.1
// Calling winId() creates the actual window and passes it 2 times into QGLWidget
// Using a custom initialization allows to make sure OpenGL is initialized after the
// QWidget creation (so the winId() is correct)
void OmGlWindow::initialize() {
  // Moving these calls into the constructor causes rendering issues on Windows
  create();
}

void OmGlWindow::renderNow(bool culling, bool offScreen) {
}

void OmGlWindow::resizeWren(int width, int height) {
  renderLater();

  emit resized();
}

QImage OmGlWindow::grabWindowBufferNow() {
  return QImage();
}

void OmGlWindow::updateScreenPixelRatio() {
  OmVideoRecorder::instance()->setScreenPixelRatio((int)devicePixelRatio());
}

void OmGlWindow::renderLater() {
  if (!mUpdatePending) {
    mUpdatePending = true;
    QApplication::postEvent(this, new QEvent(QEvent::UpdateRequest));
  }
}

bool OmGlWindow::event(QEvent *event) {
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

void OmGlWindow::flipAndScaleDownImageBuffer(const unsigned char *source, unsigned char *destination, int sourceWidth,
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

QSize OmGlWindow::minimumSize() const {
  return QSize(1, 1);
}

QSize OmGlWindow::sizeHint() const {
  return QSize(400, 400);
}

void OmGlWindow::setVideoStreamingServer(OmMultimediaStreamingServer *streamingServer) {
  mVideoStreamingServer = streamingServer;
  connect(mVideoStreamingServer, &OmMultimediaStreamingServer::imageRequested, this, &OmGlWindow::feedMultimediaStreamer);
}

void OmGlWindow::feedMultimediaStreamer() {
  renderNow();
}
