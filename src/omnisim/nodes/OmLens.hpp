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

#ifndef OM_LENS_HPP
#define OM_LENS_HPP

#include "OmBaseNode.hpp"
#include "OmSFVector2.hpp"

class OmLens : public OmBaseNode {
  Q_OBJECT

public:
  explicit OmLens(OmTokenizer *tokenizer = NULL);
  OmLens(const OmLens &other);
  explicit OmLens(const OmNode &other);
  virtual ~OmLens() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_LENS; }
  void preFinalize() override;
  void postFinalize() override;

  // getters
  const OmVector2 &center() const { return mCenter->value(); }
  const OmVector2 &radialCoefficients() const { return mRadialCoefficients->value(); }
  const OmVector2 &tangentialCoefficients() const { return mTangentialCoefficients->value(); }

signals:
  void centerChanged();
  void radialCoefficientsChanged();
  void tangentialCoefficientsChanged();

private:
  OmLens &operator=(const OmLens &);  // non copyable
  OmNode *clone() const override { return new OmLens(*this); }

  void init();

  OmSFVector2 *mCenter;
  OmSFVector2 *mRadialCoefficients;
  OmSFVector2 *mTangentialCoefficients;

private slots:
  void updateCenter();
  void updateRadialCoefficients();
  void updateTangentialCoefficients();
};

#endif
