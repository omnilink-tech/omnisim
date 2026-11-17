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

#ifndef OM_RENDERING_DEVICE_HPP
#define OM_RENDERING_DEVICE_HPP

#include <QtCore/QObject>

#include "OmSolidDevice.hpp"

class OmWrenTextureOverlay;
class OmSFDouble;
class OmSFInt;
class OmSFVector2;

class OmRenderingDevice : public OmSolidDevice {
  Q_OBJECT

public:
  virtual ~OmRenderingDevice() override;

  // reimplemented public functions
  void preFinalize() override;
  void postFinalize() override;

  // overlay related functions
  void toggleOverlayVisibility(bool enabled, bool emitSignal = false);
  void moveWindow(int dx, int dy);
  void setPixelSize(double pixelSize);

  QStringList perspective() const;
  void restorePerspective(QStringList &perspective);
  bool isOverlayEnabled() const;
  bool isWindowActive() const { return mIsExternalWindowEnabled; }
  double pixelSize() const;

  // external window
  // D1.4 (WREN deletion): the GL texture ids died with the WREN overlay textures. The
  // accessors survive for the pop-out window API but always answer 0 now; the window
  // repaints from the CPU image buffer instead of sharing a GL texture.
  virtual int textureGLId() const;
  virtual int backgroundTextureGLId() const;
  virtual int maskTextureGLId() const;
  virtual int foregroundTextureGLId() const;
  virtual void enableExternalWindow(bool enabled);

  // getters
  virtual int width() const;
  virtual int height() const;
  OmWrenTextureOverlay *overlay() const { return mOverlay; }
  virtual QString pixelInfo(int x, int y) const = 0;

  bool hasBeenSetup() const { return mHasBeenSetup; }

  // static functions
  static OmRenderingDevice *fromMousePosition(int x, int y);
  static const QList<OmRenderingDevice *> &renderingDevices() { return cRenderingDevices; }

  enum TextureRole { BACKGROUND_TEXTURE = 0, MAIN_TEXTURE, MASK_TEXTURE, FOREGROUND_TEXTURE };

signals:
  void overlayVisibilityChanged(bool visible);
  void overlayStatusChanged(bool enabled);
  void textureUpdated();
  // D1.4: WREN texture ids no longer exist; emitters may only send 0 (kept for the
  // pop-out-window signal contract).
  void textureIdUpdated(int textureGLID, TextureRole role);
  void restoreWindowPerspective(const OmRenderingDevice *device, const QStringList &perspective);
  void closeWindow();

protected:
  // all constructors are reserved for derived classes only
  OmRenderingDevice(const QString &modelName, OmTokenizer *tokenizer);
  OmRenderingDevice(const OmRenderingDevice &other);
  OmRenderingDevice(const OmNode &other);

  // WREN Data
  OmWrenTextureOverlay *mOverlay;

  // setup functions
  virtual void setup();

  virtual void createWrenOverlay() = 0;  // not very useful: this function is not called in a polymorphical way

  bool areOverlaysEnabled() const;  // global preferences value

protected slots:
  virtual void updateWidth();
  virtual void updateHeight();

private:
  OmRenderingDevice &operator=(const OmRenderingDevice &);  // non copyable
  void init();

  // user accessible fields
  OmSFInt *mWidth;
  OmSFInt *mHeight;

  // values just after the setup
  int mSetupWidth;
  int mSetupHeight;

  // private stuff
  bool mHasBeenSetup;
  bool mIsExternalWindowEnabled;

  // static variables
  static QList<OmRenderingDevice *> cRenderingDevices;  // list of the current devices rendering in the 3D-view; used by
                                                        // OmView3D after each resize event to reset texture overlays positions
};

#endif
