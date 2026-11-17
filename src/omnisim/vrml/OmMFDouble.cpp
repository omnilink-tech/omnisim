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

#include "OmMFDouble.hpp"
#include "OmToken.hpp"
#include "OmTokenizer.hpp"

void OmMFDouble::readAndAddItem(OmTokenizer *tokenizer, const QString &worldPath) {
  mVector.append(tokenizer->nextToken()->toDouble());
}

void OmMFDouble::clear() {
  if (mVector.size() > 0) {
    mVector.clear();
    emit changed();
    emit cleared();  // notify that all children have been removed
  }
}

void OmMFDouble::copyItemsTo(double values[], int max) const {
  if (max == -1 || max > mVector.size())
    max = mVector.size();

  memcpy(values, mVector.constData(), max * sizeof(double));
}

void OmMFDouble::findMinMax(double *min, double *max) const {
  if (mVector.isEmpty())
    return;

  *min = *max = mVector[0];
  foreach (double d, mVector) {
    *min = qMin(*min, d);
    *max = qMax(*max, d);
  }
}

void OmMFDouble::setItem(int index, double value) {
  assert(index >= 0 && index < size());
  if (mVector[index] != value) {
    mVector[index] = value;
    emit itemChanged(index);
    emit changed();
  }
}

void OmMFDouble::setAllItems(const double *values) {
  const int vectorSize = mVector.size();
  bool vectorHasChanged = false;
  for (int index = 0; index < vectorSize; index++) {
    if (mVector[index] != values[index]) {
      mVector[index] = values[index];
      emit itemChanged(index);
      vectorHasChanged = true;
    }
  }

  if (vectorHasChanged)
    emit changed();
}

void OmMFDouble::multiplyAllItems(double factor) {
  if (factor == 1.0)
    return;

  const int vectorSize = mVector.size();
  for (int index = 0; index < vectorSize; index++) {
    const double previousValue = mVector[index];
    if (previousValue != 0.0) {
      mVector[index] = factor * previousValue;
      emit itemChanged(index);
    }
  }

  emit changed();
}

void OmMFDouble::addItem(double value) {
  mVector.append(value);
  emit itemInserted(mVector.size() - 1);
  emit changed();
}

void OmMFDouble::insertDefaultItem(int index) {
  assert(index >= 0 && index <= size());
  mVector.insert(mVector.begin() + index, defaultNewVariant().toDouble());
  emit itemInserted(index);
  emit changed();
}

void OmMFDouble::insertItem(int index, double value) {
  assert(index >= 0 && index <= size());
  mVector.insert(mVector.begin() + index, value);
  emit itemInserted(index);
  emit changed();
}

void OmMFDouble::removeItem(int index) {
  assert(index >= 0 && index < size());
  mVector.erase(mVector.begin() + index);
  emit itemRemoved(index);
  emit changed();
}

OmMFDouble &OmMFDouble::operator=(const OmMFDouble &other) {
  if (mVector == other.mVector)
    return *this;

  mVector = other.mVector;
  emit changed();
  return *this;
}

bool OmMFDouble::equals(const OmValue *other) const {
  const OmMFDouble *that = dynamic_cast<const OmMFDouble *>(other);
  return that && *this == *that;
}

void OmMFDouble::copyFrom(const OmValue *other) {
  const OmMFDouble *that = dynamic_cast<const OmMFDouble *>(other);
  *this = *that;
}

bool OmMFDouble::smallSeparator(int i) const {
  return (i % 10 != 0 || OmMultipleValue::smallSeparator(i));  // 10 integers per line
}
