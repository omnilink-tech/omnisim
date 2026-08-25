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

#ifndef OM_RENDERING_DEVICE_WINDOW_HPP
#define OM_RENDERING_DEVICE_WINDOW_HPP

//
// The pop-out ("external") window of a rendering device (Camera / RangeFinder / Display).
//
// D1.4 (WREN deletion): it used to blit the device's WREN GL texture by id
// (device->textureGLId(), shared GL context). Those textures are gone, so it now owns ONE
// GL texture and re-uploads it from the device's CPU image buffer
// (overlay()->sourceData()/sourceWidth()/sourceHeight()/textureType()) on every repaint --
// the same source OmHudOverlay draws the inset overlays from.
//

#include "OmRenderingDevice.hpp"

#include <QtGui/QWindow>
#include <QtOpenGL/QOpenGLFunctions_3_3_Core>

#ifdef _WIN32
#include <windows.h>
#endif

class QOpenGLContext;
class QOpenGLShaderProgram;

class OmAbstractCamera;

class OmRenderingDeviceWindow : public QWindow {
  Q_OBJECT

public:
  explicit OmRenderingDeviceWindow(OmRenderingDevice *device);
  ~OmRenderingDeviceWindow();

  OmRenderingDevice *device() const { return mDevice; }
  int deviceId() const;

  QStringList perspective() const;
  void restorePerspective(const QStringList &perspective);

  static void storeOpenGLContext(QOpenGLContext *context);

protected:
  bool event(QEvent *event) override;

private:
  static QOpenGLContext *cMainOpenGLContext;

  QOpenGLContext *mContext;
  QOpenGLShaderProgram *mProgram;
  OmRenderingDevice *mDevice;
  OmAbstractCamera *mAbstractCamera;
  // D1.4: the window's OWN image texture, re-uploaded from the device's CPU buffer each
  // repaint (the shared WREN GL texture ids are gone).
  GLuint mImageTextureId;
  int mImageTextureWidth;
  int mImageTextureHeight;
  GLuint mMaxRangeUniform;
  GLuint mImageUniform;
  GLuint mVaoId;
  GLuint *mVboId;
  bool mInitialized;
  QRect mPreviousGeometry;

  bool mUpdateRequested;
  bool mShowOverlayOnClose;

  void initialize();
  void render();

private slots:
  void renderNow();
  void requestUpdate();
  void listenToBackgroundImageChanges(const OmRenderingDevice *previousAttachedDevice,
                                      const OmRenderingDevice *newAttachedDevice);
  void closeFromMainWindow();
};

#endif
