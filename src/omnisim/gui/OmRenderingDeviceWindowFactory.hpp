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

#ifndef OM_RENDERING_WINDOW_FACTORY_HPP
#define OM_RENDERING_WINDOW_FACTORY_HPP

#include <QtCore/QObject>

class QOpenGLContext;

class OmPerspective;
class OmRenderingDevice;
class OmRenderingDeviceWindow;

class OmRenderingDeviceWindowFactory : public QObject {
  Q_OBJECT

public:
  static OmRenderingDeviceWindowFactory *instance();
  static void deleteInstance();
  static void reset();
  static void storeOpenGLContext(QOpenGLContext *context);

  void showWindowForDevice(OmRenderingDevice *device);

  // connect signal to restore perspective
  void listenToRenderingDevice(const OmRenderingDevice *device) const;
  QStringList windowPerspective(const OmRenderingDevice *device);
  void saveWindowsPerspective(OmPerspective &perspective);

  // set if windows are enabled (for example for FAST mode)
  // if false then hide windows and restore them next time this flag is set to true
  void setWindowsEnabled(bool enabled);

private:
  OmRenderingDeviceWindowFactory();
  ~OmRenderingDeviceWindowFactory();

  OmRenderingDeviceWindow *getWindowForDevice(OmRenderingDevice *device, bool createIfNeeded);

  QList<OmRenderingDeviceWindow *> mWindowsList;
  QList<OmRenderingDeviceWindow *> mActiveWindowsList;
  static OmRenderingDeviceWindowFactory *cInstance;

private slots:
  void restoreWindowPerspective(const OmRenderingDevice *device, const QStringList &perspective);
  void deleteWindow();
};

#endif
