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

#ifndef OM_FOCUS_HPP
#define OM_FOCUS_HPP

#include "OmBaseNode.hpp"
#include "OmSFDouble.hpp"

class OmFocus : public OmBaseNode {
  Q_OBJECT

public:
  explicit OmFocus(OmTokenizer *tokenizer = NULL);
  OmFocus(const OmFocus &other);
  explicit OmFocus(const OmNode &other);
  virtual ~OmFocus() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_FOCUS; }
  void preFinalize() override;
  void postFinalize() override;

  // getters
  double focalDistance() const { return mFocalDistance->value(); }
  double focalLength() const { return mFocalLength->value(); }
  double minFocalDistance() const { return mMinFocalDistance->value(); }
  double maxFocalDistance() const { return mMaxFocalDistance->value(); }

  // setters
  void setFocalDistance(double focalDistance) { mFocalDistance->setValue(focalDistance); }

signals:
  void focusSettingsChanged();

private:
  OmFocus &operator=(const OmFocus &);  // non copyable
  OmNode *clone() const override { return new OmFocus(*this); }

  void init();

  OmSFDouble *mFocalDistance;
  OmSFDouble *mFocalLength;
  OmSFDouble *mMinFocalDistance;
  OmSFDouble *mMaxFocalDistance;

private slots:
  void updateFocalDistance();
  void updateFocalLength();
  void updateMinFocalDistance();
  void updateMaxFocalDistance();
};

#endif
