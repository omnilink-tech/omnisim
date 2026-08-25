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

//
//  OmSolidReference.hpp
//

// Node class representing a pointer to a Solid node
// It is used in the SFNode endPoint of a Joint to allow mechanical loop

#ifndef OM_SOLID_REFERENCE_HPP
#define OM_SOLID_REFERENCE_HPP

#include <QtCore/QPointer>
#include "OmBaseNode.hpp"
#include "OmSFString.hpp"

class OmSolid;

class OmSolidReference : public OmBaseNode {
  Q_OBJECT

public:
  explicit OmSolidReference(OmTokenizer *tokenizer = NULL);
  OmSolidReference(const OmSolidReference &other);
  explicit OmSolidReference(const OmNode &other);
  virtual ~OmSolidReference() override;

  int nodeType() const override { return WB_NODE_SOLID_REFERENCE; }
  void postFinalize() override;

  QPointer<OmSolid> solid() const { return mSolid; }
  const QString &name() const { return mName->value(); }

  void updateName();
  bool pointsToStaticEnvironment() const { return mName->value() == STATIC_ENVIRONMENT; }
  static const QString STATIC_ENVIRONMENT;

  QList<const OmBaseNode *> findClosestDescendantNodesWithDedicatedWrenNode() const override;

  QString endPointName() const override;

signals:
  void changed();

private:
  OmSolidReference &operator=(const OmSolidReference &);  // non copyable
  OmSFString *mName;
  QPointer<OmSolid> mSolid;

private slots:
  void init();
};

#endif
