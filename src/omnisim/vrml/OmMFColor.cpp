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

#include "OmMFColor.hpp"
#include "OmToken.hpp"
#include "OmTokenizer.hpp"

void OmMFColor::readAndAddItem(OmTokenizer *tokenizer, const QString &worldPath) {
  const double r = tokenizer->nextToken()->toDouble();
  const double g = tokenizer->nextToken()->toDouble();
  const double b = tokenizer->nextToken()->toDouble();
  OmRgb color(r, g, b);
  if (color.clampValuesIfNeeded())
    tokenizer->reportError(
      tr("Expected positive color values in range [0.0, 1.0], found [%1 %2 %3]. MFColor field item %4 reset to [%5 %6 %7]")
        .arg(r)
        .arg(g)
        .arg(b)
        .arg(mVector.size())
        .arg(color.red())
        .arg(color.green())
        .arg(color.blue()));
  mVector.append(color);
}

void OmMFColor::clear() {
  if (!mVector.empty()) {
    mVector.clear();
    emit changed();
    emit cleared();  // notify that all children have been removed
  }
}

void OmMFColor::insertDefaultItem(int index) {
  assert(index >= 0 && index <= size());
  mVector.insert(mVector.begin() + index, defaultNewVariant().toColor());
  emit itemInserted(index);
  emit changed();
}

void OmMFColor::removeItem(int index) {
  assert(index >= 0 && index < size());
  mVector.erase(mVector.begin() + index);
  emit itemRemoved(index);
  emit changed();
}

void OmMFColor::setItem(int index, const OmRgb &value, bool signal) {
  assert(index >= 0 && index < size());
  if (mVector[index] != value) {
    mVector[index] = value;
    if (signal) {
      emit itemChanged(index);
      emit changed();
    }
  }
}

void OmMFColor::addItem(const OmRgb &value) {
  mVector.append(value);
  emit itemInserted(mVector.size() - 1);
  emit changed();
}

void OmMFColor::insertItem(int index, const OmRgb &value) {
  assert(index >= 0 && index <= size());
  mVector.insert(mVector.begin() + index, value);
  emit itemInserted(index);
  emit changed();
}

OmMFColor &OmMFColor::operator=(const OmMFColor &other) {
  if (mVector == other.mVector)
    return *this;

  mVector = other.mVector;
  emit changed();
  return *this;
}

bool OmMFColor::equals(const OmValue *other) const {
  const OmMFColor *that = dynamic_cast<const OmMFColor *>(other);
  return that && *this == *that;
}

void OmMFColor::copyFrom(const OmValue *other) {
  const OmMFColor *that = dynamic_cast<const OmMFColor *>(other);
  *this = *that;
}
