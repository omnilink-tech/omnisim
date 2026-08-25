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

#ifndef OM_DAMPING_HPP
#define OM_DAMPING_HPP

#include "OmBaseNode.hpp"
#include "OmSFDouble.hpp"

class OmDamping : public OmBaseNode {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmDamping(OmTokenizer *tokenizer = NULL);
  OmDamping(const OmDamping &other);
  explicit OmDamping(const OmNode &other);
  virtual ~OmDamping() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_DAMPING; }
  void preFinalize() override;
  void postFinalize() override;

  // field accessors
  double linear() const { return mLinear->value(); }
  double angular() const { return mAngular->value(); }

signals:
  void changed();

private:
  // user accessible fields
  OmSFDouble *mLinear;
  OmSFDouble *mAngular;

  OmDamping &operator=(const OmDamping &);  // non copyable
  OmNode *clone() const override { return new OmDamping(*this); }
  void init();

private slots:
  void updateLinear();
  void updateAngular();
};

#endif
