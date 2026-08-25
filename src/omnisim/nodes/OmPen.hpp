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

#ifndef OM_PEN_HPP
#define OM_PEN_HPP

#include "OmSolidDevice.hpp"

class OmPaintTexture;

class OmPen : public OmSolidDevice {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmPen(OmTokenizer *tokenizer = NULL);
  OmPen(const OmPen &other);
  explicit OmPen(const OmNode &other);
  virtual ~OmPen() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_PEN; }
  void preFinalize() override;
  void handleMessage(QDataStream &) override;
  void prePhysicsStep(double ms) override;
  void reset(const QString &id) override;

private:
  OmSFBool *mWrite;
  OmSFColor *mInkColor;
  OmSFDouble *mInkDensity;
  OmSFDouble *mLeadSize;
  OmSFDouble *mMaxDistance;

  OmPen &operator=(const OmPen &);  // non copyable
  OmNode *clone() const override { return new OmPen(*this); }
  void init();

  OmPaintTexture *mLastPaintTexture;
};

#endif  // OM_PEN_HPP
