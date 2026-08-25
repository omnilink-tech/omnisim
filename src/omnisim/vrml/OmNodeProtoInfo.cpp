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

#include "OmNodeProtoInfo.hpp"

#include "OmField.hpp"

static const OmFieldReference INVALID_FIELD_REFERENCE = {QString(), NULL};

OmNodeProtoInfo::OmNodeProtoInfo(const QString &modelName, const QList<OmField *> &parameters) : mModelName(modelName) {
  foreach (OmField *field, parameters) {
    OmFieldReference ref;
    ref.name = field->name();
    ref.actualField = field;
    mParameters.append(ref);
  }
}

OmNodeProtoInfo::OmNodeProtoInfo(const OmNodeProtoInfo &other) : mModelName(other.mModelName), mParameters(other.mParameters) {
}

const OmFieldReference &OmNodeProtoInfo::findFieldByIndex(int index) const {
  if (index < 0 || index >= mParameters.size())
    return INVALID_FIELD_REFERENCE;
  return mParameters.at(index);
}

int OmNodeProtoInfo::findFieldIndex(const QString &name) const {
  for (int i = 0; i < mParameters.size(); ++i)
    if (mParameters.at(i).name == name)
      return i;
  return -1;
}

void OmNodeProtoInfo::redirectFields(const OmField *oldField, OmField *newField) {
  for (OmFieldReference &ref : mParameters)
    if (ref.actualField == oldField)
      ref.actualField = newField;
}
