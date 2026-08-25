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

#ifndef OM_ZOOM_HPP
#define OM_ZOOM_HPP

#include "OmBaseNode.hpp"
#include "OmSFDouble.hpp"

class OmZoom : public OmBaseNode {
  Q_OBJECT

public:
  explicit OmZoom(OmTokenizer *tokenizer = NULL);
  OmZoom(const OmZoom &other);
  explicit OmZoom(const OmNode &other);
  virtual ~OmZoom() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_ZOOM; }
  void preFinalize() override;
  void postFinalize() override;

  // getters
  double minFieldOfView() const { return mMinFieldOfView->value(); }
  double maxFieldOfView() const { return mMaxFieldOfView->value(); }

private:
  OmZoom &operator=(const OmZoom &);  // non copyable
  OmNode *clone() const override { return new OmZoom(*this); }

  void init();

  OmSFDouble *mMinFieldOfView;
  OmSFDouble *mMaxFieldOfView;

private slots:
  void updateMinFieldOfView();
  void updateMaxFieldOfView();
};

#endif
