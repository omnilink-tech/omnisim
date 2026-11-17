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

#include "OmSFVector2.hpp"
#include "OmToken.hpp"
#include "OmTokenizer.hpp"

void OmSFVector2::readSFVector2(OmTokenizer *tokenizer, const QString &worldPath) {
  try {
    double xCoordinate = tokenizer->nextToken()->toDouble();
    double yCoordinate = tokenizer->nextToken()->toDouble();
    mValue.setXy(xCoordinate, yCoordinate);
    mValue.clamp();
  } catch (...) {
    tokenizer->reportError(tr("Expected floating point value, found %1").arg(tokenizer->lastWord()), tokenizer->lastToken());
    tokenizer->ungetToken();  // unexpected token: keep the tokenizer coherent
    throw 0;                  // report the exception
  }
}

void OmSFVector2::setValue(const OmVector2 &v) {
  if (mValue == v)
    return;

  mValue = v;
  emit changed();
}

void OmSFVector2::setValue(double x, double y) {
  if (mValue == OmVector2(x, y))
    return;

  mValue.setXy(x, y);
  emit changed();
}

void OmSFVector2::setValue(const double xy[2]) {
  if (mValue == OmVector2(xy))
    return;

  mValue.setXy(xy);
  emit changed();
}

void OmSFVector2::setValueFromOmniSim(const OmVector2 &v) {
  if (mValue == v)
    return;

  mValue = v;
  emit changedByOmniSim();
}

void OmSFVector2::mult(double factor) {
  if (factor == 1.0)
    return;

  mValue *= factor;
  emit changed();
}

void OmSFVector2::setComponent(int index, double d) {
  if (component(index) == d)
    return;

  mValue[index] = d;
  emit changed();
}

OmSFVector2 &OmSFVector2::operator=(const OmSFVector2 &other) {
  if (mValue == other.mValue)
    return *this;

  mValue = other.mValue;
  emit changed();
  return *this;
}

bool OmSFVector2::equals(const OmValue *other) const {
  const OmSFVector2 *that = dynamic_cast<const OmSFVector2 *>(other);
  return that && *this == *that;
}

void OmSFVector2::copyFrom(const OmValue *other) {
  const OmSFVector2 *that = dynamic_cast<const OmSFVector2 *>(other);
  *this = *that;
}
