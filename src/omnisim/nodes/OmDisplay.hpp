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

#ifndef OM_DISPLAY_HPP
#define OM_DISPLAY_HPP

#include "OmRenderingDevice.hpp"

class OmCamera;
class OmDisplayFont;
class OmDisplayImage;
class OmImageTexture;

class QDataStream;

class OmDisplay : public OmRenderingDevice {
  Q_OBJECT
public:
  // constructors and destructor
  explicit OmDisplay(OmTokenizer *tokenizer = NULL);
  OmDisplay(const OmDisplay &other);
  explicit OmDisplay(const OmNode &other);
  virtual ~OmDisplay() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_DISPLAY; }
  void preFinalize() override;
  void handleMessage(QDataStream &) override;
  void writeAnswer(OmDataStream &) override;
  void writeConfigure(OmDataStream &) override;
  QString pixelInfo(int x, int y) const override;
  void createWrenObjects() override;
  void postPhysicsStep() override;
  void reset(const QString &id) override;
  void enableExternalWindow(bool enabled) override;

  OmCamera *const attachedCamera() const { return mAttachedCamera; }

signals:
  void attachedCameraChanged(const OmRenderingDevice *previousAttachedDevice, const OmRenderingDevice *newAttachedDevice);

protected:
  void setup() override;

private:
  // getters
  int red(int x, int y) const { return shiftedChannel(x, y, 16); }
  int green(int x, int y) const { return shiftedChannel(x, y, 8); }
  int blue(int x, int y) const { return shiftedChannel(x, y, 0); }
  int alpha(int x, int y) const { return shiftedChannel(x, y, 24); }
  int shiftedChannel(int x, int y, int shift) const;

  OmDisplay &operator=(const OmDisplay &);  // non copyable
  OmNode *clone() const override { return new OmDisplay(*this); }
  void init();

  void clearImages();

  bool isUpdateRequired() { return mUpdateRequired; }
  void createWrenOverlay() override;
  void setTransparentTextureIfNeeded();

  void attachCamera(WbDeviceTag cameraTag);

  void drawPixel(int x, int y);
  void fastDrawPixel(int offset);
  void drawLine(int x0, int y0, int x1, int y1);
  void drawRectangle(int x, int y, int w, int h, bool fill);
  void drawOval(int cx, int cy, int a, int b, bool fill);
  int drawChar(unsigned long c, int x, int y);  // return character width in pixels
  void setFont(char *font, unsigned int size);
  void drawText(const char *txt, int x, int y);
  void drawPolygon(const int *px, const int *py, int size, bool fill);
  unsigned int *imageCopy(short int x, short int y, short int &w,
                          short int &h);  // return copied data and clipped width and height
  void imagePaste(int id, int x, int y, bool blend);
  void imageLoad(int id, int w, int h, void *data, int format);
  void imageDelete(int id);
  OmDisplayImage *imageFind(int id);
  static int channelNumberFromPixelFormat(int pixelFormat);

  unsigned int *mImage;  // BGRA 8+8+8+8
  unsigned int mColor;   // 0RGB 8+8+8
  unsigned char mAlpha;
  unsigned char mOpacity;
  bool mAntiAliasing;
  bool mIsTextureTransparent;
  OmDisplayFont *mDisplayFont;
  QVector<OmDisplayImage *> mImages;      // contains the list of images stored by load or copy
  QVector<OmDisplayImage *> mSaveOrders;  // contains the list of images' id to send to the controller
  bool mRequestImages;                    // set when the controller request the state of the display
  bool mUpdateRequired;
  OmCamera *mAttachedCamera;
  bool mNeedToSetExternalTextures;

  void findImageTextures();
  void findImageTextures(OmGroup *group);
  void clearImageTextures();
  bool isAnyImageTextureFound() { return (bool)mImageTextures.size(); }
  QList<OmImageTexture *> mImageTextures;

private slots:
  void removeImageTexture(QObject *object);
  void removeExternalTextures();
  void detachCamera();
};

#endif  // OM_DISPLAY_HPP
