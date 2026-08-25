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

#ifndef OM_FOG_HPP
#define OM_FOG_HPP

#include "OmBaseNode.hpp"

// TODO: only the first Fog node should be taken into account

class OmFog : public OmBaseNode {
  Q_OBJECT

public:
  static int numberOfFogInstances() { return cFogList.count(); }
  static OmFog *fogInstance() { return cFogList.count() > 0 ? cFogList.first() : NULL; }

  // constructors and destructor
  explicit OmFog(OmTokenizer *tokenizer = NULL);
  OmFog(const OmFog &other);
  explicit OmFog(const OmNode &other);
  virtual ~OmFog() override;

  // D1.4: engine-side fog curve, latched from the 'fogType' field (was WREN's WrSceneFogType).
  enum FogType { FOG_TYPE_NONE, FOG_TYPE_LINEAR, FOG_TYPE_EXPONENTIAL, FOG_TYPE_EXPONENTIAL2 };

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_FOG; }
  void preFinalize() override;
  void postFinalize() override;

  int mode() const { return mFogTypeMode; }

  // Additive getters for the wgpu main view (it applies fog per-pixel in its scene shader rather
  // than through WREN's GL fog state). Color/range read the user-authored fields directly.
  const OmSFColor *fogColor() const { return mColor; }
  const OmSFDouble *fogVisibilityRange() const { return mVisibilityRange; }

  QStringList fieldsToSynchronizeWithW3d() const override;

signals:
  void modeChanged();

private:
  static QList<OmFog *> cFogList;

  OmFog &operator=(const OmFog &);  // non copyable
  // reimplemented functions
  OmNode *clone() const override { return new OmFog(*this); }

  // other functions
  void init();
  bool isFirstInstance() const { return cFogList.first() == this; }
  void updateAndConnectIfNeeded();

  // user accessible fields
  OmSFColor *mColor;
  OmSFString *mFogType;
  OmSFDouble *mVisibilityRange;

  // other variables
  FogType mFogTypeMode;

private slots:
  void updateColor();
  void updateFogType();
  void updateVisibilityRange();
};

#endif
