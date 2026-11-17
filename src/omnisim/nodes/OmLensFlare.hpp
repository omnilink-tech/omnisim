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

#ifndef OM_LENS_FLARE_HPP
#define OM_LENS_FLARE_HPP

#include <QtCore/QMap>

#include "OmBaseNode.hpp"
#include "OmVector3.hpp"

class OmSFDouble;

class OmLensFlare : public OmBaseNode {
  Q_OBJECT

public:
  explicit OmLensFlare(OmTokenizer *tokenizer = NULL);
  OmLensFlare(const OmLensFlare &other);
  explicit OmLensFlare(const OmNode &other);
  virtual ~OmLensFlare() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_LENS; }
  void postFinalize() override;

  // D1.4: the WREN lens-flare post-processing effect is retired (E1-style); the node keeps
  // its field parsing/validation only. detachFromViewport() remains as a no-op for callers.
  void detachFromViewport();

private:
  OmLensFlare &operator=(const OmLensFlare &);  // non copyable
  OmNode *clone() const override { return new OmLensFlare(*this); }

  void init();

  // user accessible fields
  OmSFDouble *mTransparency;
  OmSFDouble *mScale;
  OmSFDouble *mBias;
  OmSFDouble *mDispersal;
  OmSFDouble *mHaloWidth;
  OmSFDouble *mChromaDistortion;
  OmSFInt *mSamples;
  OmSFInt *mBlurIterations;

private slots:
  void updateTransparency();
  void updateScale();
  void updateBias();
  void updateDispersal();
  void updateHaloWidth();
  void updateChromaDistortion();
  void updateSamples();
  void updateBlur();
};

#endif
