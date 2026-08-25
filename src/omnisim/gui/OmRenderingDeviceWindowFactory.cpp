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

#include "OmRenderingDeviceWindowFactory.hpp"

#include "OmPerspective.hpp"
#include "OmRenderingDevice.hpp"
#include "OmRenderingDeviceWindow.hpp"

#include <cassert>

OmRenderingDeviceWindowFactory *OmRenderingDeviceWindowFactory::cInstance = NULL;

OmRenderingDeviceWindowFactory *OmRenderingDeviceWindowFactory::instance() {
  if (!cInstance)
    cInstance = new OmRenderingDeviceWindowFactory();

  return cInstance;
}

void OmRenderingDeviceWindowFactory::deleteInstance() {
  delete cInstance;
  cInstance = NULL;
}

void OmRenderingDeviceWindowFactory::reset() {
  delete cInstance;
  cInstance = new OmRenderingDeviceWindowFactory();
}

void OmRenderingDeviceWindowFactory::storeOpenGLContext(QOpenGLContext *context) {
  OmRenderingDeviceWindow::storeOpenGLContext(context);
}

OmRenderingDeviceWindowFactory::OmRenderingDeviceWindowFactory() {
  QList<OmRenderingDevice *> devices = OmRenderingDevice::renderingDevices();
  for (int i = 0; i < devices.size(); ++i)
    connect(devices[i], &OmRenderingDevice::restoreWindowPerspective, this,
            &OmRenderingDeviceWindowFactory::restoreWindowPerspective, Qt::UniqueConnection);
}

OmRenderingDeviceWindowFactory::~OmRenderingDeviceWindowFactory() {
  for (int i = 0; i < mWindowsList.size(); ++i) {
    mWindowsList[i]->setVisible(false);
    delete mWindowsList[i];
  }
  mWindowsList.clear();
  mActiveWindowsList.clear();
}

void OmRenderingDeviceWindowFactory::listenToRenderingDevice(const OmRenderingDevice *device) const {
  connect(device, &OmRenderingDevice::restoreWindowPerspective, this, &OmRenderingDeviceWindowFactory::restoreWindowPerspective,
          Qt::UniqueConnection);
}

void OmRenderingDeviceWindowFactory::showWindowForDevice(OmRenderingDevice *device) {
  OmRenderingDeviceWindow *window = getWindowForDevice(device, true);
  window->show();
}

void OmRenderingDeviceWindowFactory::saveWindowsPerspective(OmPerspective &perspective) {
  for (int i = 0; i < mWindowsList.size(); ++i) {
    if (mWindowsList[i]->isVisible()) {
      const OmRenderingDeviceWindow *window = mWindowsList[i];
      QStringList devicePerspective(window->device()->perspective());
      devicePerspective << window->perspective();
      perspective.setRenderingDevicePerspective(window->device()->computeShortUniqueName(), devicePerspective);
    }
  }
}

QStringList OmRenderingDeviceWindowFactory::windowPerspective(const OmRenderingDevice *device) {
  const OmRenderingDeviceWindow *window = getWindowForDevice(const_cast<OmRenderingDevice *>(device), false);
  if (window) {
    QStringList perspective(window->device()->perspective());
    perspective << window->perspective();
    return perspective;
  }
  return QStringList();
}

void OmRenderingDeviceWindowFactory::restoreWindowPerspective(const OmRenderingDevice *device, const QStringList &perspective) {
  if (perspective.size() < 5)
    // invalid window perspective
    return;

  bool valid = false;
  for (int i = 0; i < 5; i++)
    valid |= perspective.at(i) != "0";
  if (!valid)
    return;  // external window disabled

  OmRenderingDeviceWindow *window = getWindowForDevice(const_cast<OmRenderingDevice *>(device), true);
  window->restorePerspective(perspective);
  window->show();
}

OmRenderingDeviceWindow *OmRenderingDeviceWindowFactory::getWindowForDevice(OmRenderingDevice *device, bool createIfNeeded) {
  for (int i = 0; i < mWindowsList.size(); ++i) {
    if (mWindowsList[i]->deviceId() == device->uniqueId())
      return mWindowsList[i];
  }

  if (!createIfNeeded)
    return NULL;

  OmRenderingDeviceWindow *window = new OmRenderingDeviceWindow(device);
  connect(device, &OmNode::isBeingDestroyed, this, &OmRenderingDeviceWindowFactory::deleteWindow);
  mWindowsList.append(window);
  return window;
}

void OmRenderingDeviceWindowFactory::setWindowsEnabled(bool enabled) {
  if (enabled == mActiveWindowsList.isEmpty())
    return;

  if (enabled) {
    for (int i = 0; i < mActiveWindowsList.size(); ++i)
      mActiveWindowsList[i]->show();
    mActiveWindowsList.clear();
  } else {
    for (int i = 0; i < mWindowsList.size(); ++i) {
      if (!mWindowsList[i]->isVisible())
        continue;
      mActiveWindowsList.append(mWindowsList[i]);
      mWindowsList[i]->close();
    }
  }
}

void OmRenderingDeviceWindowFactory::deleteWindow() {
  OmNode *node = dynamic_cast<OmNode *>(sender());
  assert(node);
  for (int i = 0; i < mWindowsList.size(); ++i) {
    if (mWindowsList[i]->deviceId() == node->uniqueId()) {
      OmRenderingDeviceWindow *window = mWindowsList[i];
      mActiveWindowsList.removeOne(window);
      mWindowsList.removeOne(window);
      delete window;
      return;
    }
  }
}
