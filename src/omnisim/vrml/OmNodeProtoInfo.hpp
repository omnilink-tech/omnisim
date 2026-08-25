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

#ifndef OM_NODE_PROTO_ANCESTRY_HPP
#define OM_NODE_PROTO_ANCESTRY_HPP

//
// Description: class containing proto-specific information about a VRML node
//

#include <QtCore/QList>
#include <QtCore/QString>

class OmField;

struct OmFieldReference {
  QString name;
  OmField *actualField;
};

class OmNodeProtoInfo {
public:
  OmNodeProtoInfo(const QString &modelName, const QList<OmField *> &parameters);
  explicit OmNodeProtoInfo(const OmNodeProtoInfo &other);

  const QString &modelName() const { return mModelName; }
  const QList<OmFieldReference> &parameters() const { return mParameters; }

  const OmFieldReference &findFieldByIndex(int index) const;
  int findFieldIndex(const QString &name) const;

  void redirectFields(const OmField *oldField, OmField *newField);

private:
  OmNodeProtoInfo &operator=(const OmNodeProtoInfo &);  // non copyable

  QString mModelName;
  QList<OmFieldReference> mParameters;
};

#endif
