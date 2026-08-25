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

#ifndef OM_RECOGNITION_HPP
#define OM_RECOGNITION_HPP

#include "OmBaseNode.hpp"
#include "OmSFBool.hpp"
#include "OmSFColor.hpp"
#include "OmSFDouble.hpp"
#include "OmSFInt.hpp"

class OmRecognition : public OmBaseNode {
  Q_OBJECT

public:
  explicit OmRecognition(OmTokenizer *tokenizer = NULL);
  OmRecognition(const OmRecognition &other);
  explicit OmRecognition(const OmNode &other);
  virtual ~OmRecognition() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_RECOGNITION; }
  void preFinalize() override;
  void postFinalize() override;

  // getters
  double maxRange() const { return mMaxRange->value(); }
  int maxObjects() const { return mMaxObjects->value(); }
  int occlusion() const { return mOcclusion->value(); }
  const OmRgb frameColor() const { return mFrameColor->value(); }
  int frameThickness() const { return mFrameThickness->value(); }
  bool segmentation() const { return mSegmentation->value(); }

  void setSegmentation(bool value) { mSegmentation->setValue(value); }

signals:
  void segmentationChanged();

private:
  OmRecognition &operator=(const OmRecognition &);  // non copyable
  OmNode *clone() const override { return new OmRecognition(*this); }

  void init();

  OmSFDouble *mMaxRange;
  OmSFInt *mMaxObjects;
  OmSFInt *mOcclusion;
  OmSFColor *mFrameColor;
  OmSFInt *mFrameThickness;
  OmSFBool *mSegmentation;

private slots:
  void updateMaxRange();
  void updateMaxObjects();
  void updateOcclusion();
  void updateFrameThickness();
};

#endif
