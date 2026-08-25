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

#ifndef OM_BACKGROUND_HPP
#define OM_BACKGROUND_HPP

#include "OmBaseNode.hpp"
#include "OmSFDouble.hpp"

class OmDownloader;
class OmRgb;
class OmSFString;


class OmBackground : public OmBaseNode {
  Q_OBJECT

public:
  static OmBackground *firstInstance() { return cBackgroundList.isEmpty() ? NULL : cBackgroundList.first(); }
  static int numberOfBackgroundInstances() { return cBackgroundList.count(); }

  // constructors and destructor
  explicit OmBackground(OmTokenizer *tokenizer = NULL);
  OmBackground(const OmBackground &other);
  explicit OmBackground(const OmNode &other);
  virtual ~OmBackground() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_BACKGROUND; }
  void downloadAssets() override;
  void preFinalize() override;
  void postFinalize() override;
  void createWrenObjects() override;

  // accessor
  OmRgb skyColor() const;
  double luminosity() const { return mLuminosity->value(); }
  // trimmed atmosphericSky preset name ("" = none) — the wgpu main view keys its procedural dome on this
  QString atmosphericSkyPreset() const;
  // decoded image-cubemap face i (WREN cube-face order +X,-X,+Y,-Y,+Z,-Z; coordinate swap and
  // per-face rotation already applied at load) or null. All six non-null = a complete image skybox.
  const QImage *cubemapTexture(int i) const { return (i >= 0 && i < 6) ? mTexture[i] : nullptr; }
  bool hasCompleteCubemap() const {
    for (int i = 0; i < 6; ++i)
      if (!mTexture[i])
        return false;
    return true;
  }


signals:
  void cubemapChanged();
  void luminosityChanged();

protected:
  void exportNodeFields(OmWriter &writer) const override;
  QStringList customExportedFields() const override;

private:
  static QList<OmBackground *> cBackgroundList;

  // reimplemented functions
  OmNode *clone() const override { return new OmBackground(*this); }

  // misc functions
  OmBackground &operator=(const OmBackground &);  // non copyable
  void init();
  bool loadTexture(int i);

  bool isFirstInstance() { return cBackgroundList.first() == this; }
  // make this the OmBackground instance in use
  void activate();
  void downloadAsset(const QString &url, int index, bool postpone);

  // user accessible fields
  OmMFColor *mSkyColor;
  OmMFString *mUrlFields[6];
  // P4a (WREN retirement): the six *IrradianceUrl fields are RETIRED — still declared in
  // Background.wrl (an undeclared field is a parse ERROR that takes a headless run's exit code to
  // 1) and still parsed here, but never downloaded or decoded. Authoring any of them draws a
  // one-time warning naming the fields; the irradiance cube is always baked from the sky itself
  // (atmosphericSky preset, the image cubemap, or skyColor).
  OmMFString *mIrradianceUrlFields[6];
  OmSFDouble *mLuminosity;
  OmSFString *mAtmosphericSky;

  // texture loading fields
  QImage *mTexture[6];
  bool mTextureHasAlpha;
  int mTextureSize;
  int mUrlCount;

  // skybox related fields

  // hdr-clearing quad-related fields


  // Hillaire 2020 atmospheric sky.  Lazily instantiated the first time
  // ``mAtmosphericSky`` resolves to a non-empty preset name; otherwise
  // stays nullptr and the cubemap path runs as before.  See T1.3 in
  // docs/developer/rendering-roadmap.md.

  OmDownloader *mDownloader[6];

  // P4a: one warning per node, shared by the load-time check (postFinalize) and the runtime
  // field-changed hook.
  bool mIrradianceRetirementWarned;

private slots:
  void updateColor();
  void updateCubemap();
  void updateLuminosity();
  void downloadUpdate();
  void warnIrradianceUrlRetired();  // P4a: named warning when any *IrradianceUrl is authored
};

#endif
