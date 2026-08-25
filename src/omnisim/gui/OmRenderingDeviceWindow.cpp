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

#include "OmRenderingDeviceWindow.hpp"

#include "OmAbstractCamera.hpp"
#include "OmCamera.hpp"
#include "OmDisplay.hpp"
#include "OmNodeUtilities.hpp"
#include "OmPerformanceLog.hpp"
#include "OmRenderingDevice.hpp"
#include "OmRobot.hpp"
#include "OmWrenRenderingContext.hpp"
#include "OmWrenTextureOverlay.hpp"

#include <QtGui/QOpenGLContext>
#include <QtOpenGL/QOpenGLShaderProgram>
#include <QtOpenGL/QOpenGLVersionFunctionsFactory>

// D1.4 (WREN deletion): this window no longer shares the main GL context's WREN textures by
// id -- it re-uploads the device's CPU image (overlay()->sourceData()) into its own texture
// on every repaint, exactly the buffer the controller reads and OmHudOverlay draws.
// D1.4 TODO(excision): the WREN-era background compositing (a Display's attached-camera
// image under the display drawing) and the Camera recognition mask/foreground layers were
// separate WREN textures and are not reproduced here yet -- the pop-out shows the device's
// own image only.

static const char *gVertexShaderSource = "#version 330\n"
                                         "layout (location = 0) in vec4 posAttr;\n"
                                         "layout (location = 1) in vec2 uvAttr;\n"
                                         "out vec2 uv0;\n"
                                         "void main() {\n"
                                         "  uv0 = uvAttr;\n"
                                         "  gl_Position = posAttr;\n"
                                         "}\n";

static const char *gStdFragmentShaderSource = "#version 330\n"
                                              "uniform sampler2D image;\n"
                                              "in vec2 uv0;\n"
                                              "out vec4 fragColor;\n"
                                              "void main() {\n"
                                              "  vec2 deviceUv0 = uv0;\n"
                                              "  fragColor = texture(image, deviceUv0);\n"
                                              "}\n";

static const char *gDepthFragmentShaderSource = "#version 330\n"
                                                "uniform sampler2D image;\n"
                                                "uniform float maxRange;\n"
                                                "in vec2 uv0;\n"
                                                "out vec4 fragColor;\n"
                                                "void main() {\n"
                                                "   float normalizedDepth = texture(image, uv0).x / maxRange;\n"
                                                "   fragColor = vec4(normalizedDepth, normalizedDepth, normalizedDepth, 1.0);\n"
                                                "}\n";

QOpenGLContext *OmRenderingDeviceWindow::cMainOpenGLContext = NULL;

void OmRenderingDeviceWindow::storeOpenGLContext(QOpenGLContext *context) {
  cMainOpenGLContext = context;
}

OmRenderingDeviceWindow::OmRenderingDeviceWindow(OmRenderingDevice *device) :
  QWindow(),
  mContext(NULL),
  mProgram(NULL),
  mDevice(device),
  mImageTextureId(0),
  mImageTextureWidth(0),
  mImageTextureHeight(0),
  mVboId(NULL),
  mInitialized(false),
  mUpdateRequested(true),
  mShowOverlayOnClose(true) {
  setSurfaceType(QWindow::OpenGLSurface);
  // D1.4: the shared main WREN GL context is gone (storeOpenGLContext() is no longer called
  // by anything) and this window does not need to share one any more -- its image texture is
  // its own, uploaded from the device's CPU buffer. Ask for a 3.3 core context of our own.
  if (cMainOpenGLContext)
    setFormat(cMainOpenGLContext->format());
  else {
    QSurfaceFormat surfaceFormat;
    surfaceFormat.setVersion(3, 3);
    surfaceFormat.setProfile(QSurfaceFormat::CoreProfile);
    setFormat(surfaceFormat);
  }

  mAbstractCamera = dynamic_cast<OmAbstractCamera *>(mDevice);
  connect(mDevice, &OmRenderingDevice::textureUpdated, this, &OmRenderingDeviceWindow::requestUpdate);
  connect(mDevice, &OmRenderingDevice::closeWindow, this, &OmRenderingDeviceWindow::closeFromMainWindow);
  connect(OmWrenRenderingContext::instance(), &OmWrenRenderingContext::mainRenderingEnded, this,
          &OmRenderingDeviceWindow::renderNow);
  const OmDisplay *display = dynamic_cast<OmDisplay *>(mDevice);
  if (display) {
    connect(display, &OmDisplay::attachedCameraChanged, this, &OmRenderingDeviceWindow::listenToBackgroundImageChanges);
    listenToBackgroundImageChanges(NULL, display->attachedCamera());
  }

  // set initial size
  double pixelSize = mDevice->pixelSize();
  const double textureWidth = mDevice->width();
  const double textureHeight = mDevice->height();
  double windowWidth = textureWidth * pixelSize;
  double windowHeight = textureHeight * pixelSize;
  QSize minSize = minimumSize();
  if (windowWidth < minSize.width()) {
    double newPixelSize = windowWidth / textureWidth;
    windowWidth = textureWidth * newPixelSize;
    windowHeight = textureHeight * newPixelSize;
  }

  if (windowHeight < minSize.height()) {
    double newPixelSize = windowHeight / textureHeight;
    windowWidth = textureWidth * newPixelSize;
    windowHeight = textureHeight * newPixelSize;
  }

  const OmRobot *const robotNode = OmNodeUtilities::findRobotAncestor(mDevice);
  assert(robotNode);
  setTitle(robotNode->name() + ": " + mDevice->name());
  resize(windowWidth, windowHeight);
}

OmRenderingDeviceWindow::~OmRenderingDeviceWindow() {
  if (!mContext)
    return;

  if (!isVisible())
    show();  // if the window is not exposed mContext->makeCurrent() doesn't work

  const bool success = mContext->makeCurrent(this);
  assert(success);
  if (!success)
    return;
  QOpenGLFunctions_3_3_Core *f = QOpenGLVersionFunctionsFactory::get<QOpenGLFunctions_3_3_Core>(mContext);
  if (mImageTextureId) {
    f->glDeleteTextures(1, &mImageTextureId);
    mImageTextureId = 0;
  }
  f->glDeleteVertexArrays(1, &mVaoId);
  f->glDeleteBuffers(2, reinterpret_cast<GLuint *>(&mVboId));
  mContext->doneCurrent();
  delete mVboId;
}

void OmRenderingDeviceWindow::initialize() {
  if (mProgram == NULL)
    mProgram = new QOpenGLShaderProgram(this);
  else
    mProgram->removeAllShaders();
  mProgram->addShaderFromSourceCode(QOpenGLShader::Vertex, gVertexShaderSource);
  if (mAbstractCamera && mAbstractCamera->isRangeFinder())
    mProgram->addShaderFromSourceCode(QOpenGLShader::Fragment, gDepthFragmentShaderSource);
  else
    mProgram->addShaderFromSourceCode(QOpenGLShader::Fragment, gStdFragmentShaderSource);
  mProgram->link();
  mMaxRangeUniform = mProgram->uniformLocation("maxRange");
  mImageUniform = mProgram->uniformLocation("image");

  if (!mDevice->hasBeenSetup())
    return;

  QOpenGLFunctions_3_3_Core *f = QOpenGLVersionFunctionsFactory::get<QOpenGLFunctions_3_3_Core>(mContext);

  // The window owns its image texture; render() re-uploads it from the device's CPU buffer.
  if (!mImageTextureId)
    f->glGenTextures(1, &mImageTextureId);
  mImageTextureWidth = 0;
  mImageTextureHeight = 0;

  static const GLfloat vertices[] = {-1.0f, -1.0f, 1.0f, 1.0f, -1.0f, 1.0f, -1.0f, -1.0f, 1.0f, -1.0f, 1.0f, 1.0f};

  // The texture is sized exactly to the device image, so the uv rectangle is the full [0,1]
  // range (the old WREN texture could be padded, hence the removed x/y factors).
  static const GLfloat texCoords[] = {0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f};

  if (!mVboId) {
    mVboId = new GLuint[2];
    f->glGenVertexArrays(1, &mVaoId);
    f->glGenBuffers(2, mVboId);
  }
  f->glBindVertexArray(mVaoId);
  f->glBindBuffer(GL_ARRAY_BUFFER, mVboId[0]);
  f->glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
  f->glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);
  f->glEnableVertexAttribArray(0);
  f->glBindBuffer(GL_ARRAY_BUFFER, mVboId[1]);
  f->glBufferData(GL_ARRAY_BUFFER, sizeof(texCoords), texCoords, GL_STATIC_DRAW);
  f->glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, 0);
  f->glEnableVertexAttribArray(1);

  mInitialized = true;
}

void OmRenderingDeviceWindow::render() {
  QOpenGLFunctions_3_3_Core *f = QOpenGLVersionFunctionsFactory::get<QOpenGLFunctions_3_3_Core>(mContext);

  const int ratio = (int)devicePixelRatio();
  f->glViewport(0, 0, width() * ratio, height() * ratio);

  f->glClear(GL_COLOR_BUFFER_BIT);

  if (!mInitialized)
    return;

  // Source: the device's CPU image buffer (the same memory the controller reads).
  const OmWrenTextureOverlay *overlay = mDevice->overlay();
  const void *data = overlay ? overlay->sourceData() : NULL;
  const int sourceWidth = overlay ? overlay->sourceWidth() : 0;
  const int sourceHeight = overlay ? overlay->sourceHeight() : 0;
  if (!data || sourceWidth <= 0 || sourceHeight <= 0)
    return;

  mProgram->bind();

  f->glBindVertexArray(mVaoId);

  if (mAbstractCamera && mAbstractCamera->isRangeFinder())
    mProgram->setUniformValue(mMaxRangeUniform, static_cast<float>(mAbstractCamera->maxRange()));
  mProgram->setUniformValue(mImageUniform, 0);

  f->glActiveTexture(GL_TEXTURE0);
  f->glBindTexture(GL_TEXTURE_2D, mImageTextureId);
  f->glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  const bool depth = overlay->textureType() == OmWrenTextureOverlay::TEXTURE_TYPE_DEPTH;
  if (sourceWidth != mImageTextureWidth || sourceHeight != mImageTextureHeight) {
    if (depth)
      f->glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, sourceWidth, sourceHeight, 0, GL_RED, GL_FLOAT, data);
    else
      f->glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, sourceWidth, sourceHeight, 0, GL_BGRA, GL_UNSIGNED_BYTE, data);
    mImageTextureWidth = sourceWidth;
    mImageTextureHeight = sourceHeight;
  } else {
    if (depth)
      f->glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, sourceWidth, sourceHeight, GL_RED, GL_FLOAT, data);
    else
      f->glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, sourceWidth, sourceHeight, GL_BGRA, GL_UNSIGNED_BYTE, data);
  }
  f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

  f->glDrawArrays(GL_TRIANGLES, 0, 6);

  mProgram->release();
}

void OmRenderingDeviceWindow::renderNow() {
  if (!mUpdateRequested || !isExposed())
    return;

  OmPerformanceLog *log = OmPerformanceLog::instance();
  if (log)
    log->startMeasure(OmPerformanceLog::DEVICE_WINDOW_RENDERING, mDevice->deviceName() + " window");

  if (!mContext) {
    mContext = new QOpenGLContext(this);
    mContext->setFormat(requestedFormat());
    if (cMainOpenGLContext)
      mContext->setShareContext(cMainOpenGLContext);
    mContext->create();
  }

#ifndef NDEBUG
  const bool success =
#endif  // NDEBUG
    mContext->makeCurrent(this);
  assert(success);

  if (!mInitialized)
    initialize();

  render();

  mContext->swapBuffers(this);

  mContext->doneCurrent();

  mUpdateRequested = false;

  if (log)
    log->stopMeasure(OmPerformanceLog::DEVICE_WINDOW_RENDERING, mDevice->deviceName() + " window");
}

bool OmRenderingDeviceWindow::event(QEvent *event) {
  switch (event->type()) {
    case QEvent::UpdateRequest:
    case QEvent::Expose:
      mUpdateRequested = true;
      renderNow();
      return true;
    case QEvent::Show:
      mShowOverlayOnClose = true;
      mDevice->enableExternalWindow(true);
      if (!mPreviousGeometry.isNull())
        setGeometry(mPreviousGeometry);
      return QWindow::event(event);
    case QEvent::Close:
      mPreviousGeometry = geometry();
      if (mShowOverlayOnClose)
        mDevice->enableExternalWindow(false);
    default:
      return QWindow::event(event);
  }
}

void OmRenderingDeviceWindow::closeFromMainWindow() {
  mShowOverlayOnClose = false;
  close();
}

void OmRenderingDeviceWindow::requestUpdate() {
  mUpdateRequested = true;
}

int OmRenderingDeviceWindow::deviceId() const {
  return mDevice->uniqueId();
}

void OmRenderingDeviceWindow::listenToBackgroundImageChanges(const OmRenderingDevice *previousAttachedDevice,
                                                             const OmRenderingDevice *newAttachedDevice) {
  if (previousAttachedDevice)
    disconnect(previousAttachedDevice, &OmRenderingDevice::textureUpdated, this, &OmRenderingDeviceWindow::requestUpdate);
  if (newAttachedDevice)
    connect(newAttachedDevice, &OmRenderingDevice::textureUpdated, this, &OmRenderingDeviceWindow::requestUpdate);
}

QStringList OmRenderingDeviceWindow::perspective() const {
  QStringList windowPerspective;
  QRect windowGeometry = geometry();
  windowPerspective << QString::number(windowGeometry.x());
  windowPerspective << QString::number(windowGeometry.y());
  windowPerspective << QString::number(windowGeometry.width());
  windowPerspective << QString::number(windowGeometry.height());

  int state = Qt::WindowNoState;
  if (windowState() & Qt::WindowMaximized)
    state &= Qt::WindowMaximized;
  else if (windowState() & Qt::WindowFullScreen)
    state &= Qt::WindowFullScreen;
  windowPerspective << QString::number(state);
  return windowPerspective;
}

void OmRenderingDeviceWindow::restorePerspective(const QStringList &perspective) {
  assert(perspective.size() >= 5);

  int x = perspective[0].toInt();
  int y = perspective[1].toInt();
  int width = perspective[2].toInt();
  int height = perspective[3].toInt();
  QRect windowGeometry(x, y, width, height);

  setGeometry(windowGeometry);
  setWindowState((Qt::WindowState)perspective[4].toInt());
}
