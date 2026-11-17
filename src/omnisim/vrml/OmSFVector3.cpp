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

#include "OmSFVector3.hpp"
#include "OmToken.hpp"
#include "OmTokenizer.hpp"

void OmSFVector3::readSFVector3(OmTokenizer *tokenizer, const QString &worldPath) {
  try {
    double xCoordinate = tokenizer->nextToken()->toDouble();
    double yCoordinate = tokenizer->nextToken()->toDouble();
    double zCoordinate = tokenizer->nextToken()->toDouble();
    mValue.setXyz(xCoordinate, yCoordinate, zCoordinate);
    mValue.clamp();
  } catch (...) {
    tokenizer->reportError(tr("Expected floating point value, found %1").arg(tokenizer->lastWord()), tokenizer->lastToken());
    tokenizer->ungetToken();  // unexpected token: keep the tokenizer coherent
    throw 0;                  // report the exception
  }
}

void OmSFVector3::setValue(const OmVector3 &v) {
  if (mValue == v)
    return;

  mValue = v;
  emit changed();
}

void OmSFVector3::setValue(double x, double y, double z) {
  if (mValue == OmVector3(x, y, z))
    return;

  mValue.setXyz(x, y, z);
  emit changed();
}

void OmSFVector3::setValue(const double xyz[3]) {
  if (mValue == OmVector3(xyz))
    return;

  mValue.setXyz(xyz);
  emit changed();
}

void OmSFVector3::setValueByUser(const OmVector3 &v, bool changedFromSupervisor) {
  if (mValue == v)
    return;

  mValue = v;
  // emit before changed() to be sure this instance is not deleted during PROTO template regeneration
  emit changedByUser(changedFromSupervisor);
  emit changed();
}

void OmSFVector3::mult(double factor) {
  if (factor == 1.0)
    return;

  mValue *= factor;
  emit changed();
}

void OmSFVector3::setComponent(int index, double d) {
  if (component(index) == d)
    return;

  mValue[index] = d;
  emit changed();
}

OmSFVector3 &OmSFVector3::operator=(const OmSFVector3 &other) {
  if (mValue == other.mValue)
    return *this;

  mValue = other.mValue;
  emit changed();
  return *this;
}

bool OmSFVector3::equals(const OmValue *other) const {
  const OmSFVector3 *that = dynamic_cast<const OmSFVector3 *>(other);
  return that && *this == *that;
}

void OmSFVector3::copyFrom(const OmValue *other) {
  const OmSFVector3 *that = dynamic_cast<const OmSFVector3 *>(other);
  *this = *that;
}
