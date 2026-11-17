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

#ifndef OM_FIELD_VALUE_RESTRICTION_HPP
#define OM_FIELD_VALUE_RESTRICTION_HPP

#include <QtCore/QString>
#include <OmNodeModel.hpp>
#include <OmProtoModel.hpp>
#include <OmVariant.hpp>

#include "../../../include/controller/c/omnisim/supervisor.h"

class OmFieldValueRestriction : public OmVariant {
  Q_OBJECT;

public:
  OmFieldValueRestriction() : OmVariant(), mAllowsSubtypes(false) {}
  explicit OmFieldValueRestriction(bool b) : OmVariant(b), mAllowsSubtypes(false) {}
  explicit OmFieldValueRestriction(int i) : OmVariant(i), mAllowsSubtypes(false) {}
  explicit OmFieldValueRestriction(double d) : OmVariant(d), mAllowsSubtypes(false) {}
  explicit OmFieldValueRestriction(const QString &s) : OmVariant(s), mAllowsSubtypes(false) {}
  explicit OmFieldValueRestriction(const OmVector2 &v) : OmVariant(v), mAllowsSubtypes(false) {}
  explicit OmFieldValueRestriction(const OmVector3 &v) : OmVariant(v), mAllowsSubtypes(false) {}
  explicit OmFieldValueRestriction(const OmRgb &c) : OmVariant(c), mAllowsSubtypes(false) {}
  explicit OmFieldValueRestriction(const OmRotation &r) : OmVariant(r), mAllowsSubtypes(false) {}
  explicit OmFieldValueRestriction(OmNode *n, bool allowsSubtypes) : OmVariant(n), mAllowsSubtypes(allowsSubtypes) {}
  explicit OmFieldValueRestriction(const OmVariant &variant, bool allowsSubtypes) :
    OmVariant(variant),
    mAllowsSubtypes(allowsSubtypes && variant.type() == WB_SF_NODE) {}
  OmFieldValueRestriction &operator=(const OmFieldValueRestriction &other);
  bool operator==(const OmFieldValueRestriction &other) const;
  bool operator!=(const OmFieldValueRestriction &other) const;

  virtual ~OmFieldValueRestriction() override {}

  const bool allowsSubtypes() const { return mAllowsSubtypes; }

  bool isVariantAccepted(const OmVariant &variant) const;
  bool isNodeAccepted(const QString &nodeModelName, const OmNodeModel *nodeModel, const QStringList &protoParentList) const;
  bool isNodeAccepted(const OmNode *node) const;
  bool isBaseNodeTypeAccepted(const OmNodeModel *actualType) const;
  // Note: This does not check if the proto's base type would be accepted by isBaseNodeTypeAccepted
  bool isProtoNodeTypeAccepted(const OmProtoModel *actualType) const;

private:
  // If the restriction is a node type, whether the node type can be a model which "extends" the requested type
  bool mAllowsSubtypes;
};

#endif
