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

#include "OmMFInt.hpp"
#include "OmToken.hpp"
#include "OmTokenizer.hpp"

void OmMFInt::readAndAddItem(OmTokenizer *tokenizer, const QString &worldPath) {
  mVector.append(tokenizer->nextToken()->toInt());
}

void OmMFInt::clear() {
  if (mVector.size() > 0) {
    mVector.clear();
    emit changed();
    emit cleared();  // notify that all children have been removed
  }
}

void OmMFInt::setItem(int index, int value) {
  assert(index >= 0 && index < size());
  if (mVector[index] != value) {
    mVector[index] = value;
    emit itemChanged(index);
    emit changed();
  }
}

void OmMFInt::addItem(int value) {
  mVector.append(value);
  emit itemInserted(mVector.size() - 1);
  emit changed();
}

void OmMFInt::insertDefaultItem(int index) {
  assert(index >= 0 && index <= size());
  mVector.insert(index, defaultNewVariant().toInt());
  emit itemInserted(index);
  emit changed();
}

void OmMFInt::insertItem(int index, int value) {
  assert(index >= 0 && index <= size());
  mVector.insert(index, value);
  emit itemInserted(index);
  emit changed();
}

void OmMFInt::removeItem(int index) {
  assert(index >= 0 && index < size());
  mVector.remove(index);
  emit itemRemoved(index);
  emit changed();
}

void OmMFInt::normalizeIndices() {
  bool modified = false;
  for (QVector<int>::iterator i = mVector.begin(); i != mVector.end(); ++i)
    if (*i < -1) {
      *i = -1;
      modified = true;
    }

  if (modified)
    emit changed();
}

OmMFInt &OmMFInt::operator=(const OmMFInt &other) {
  if (mVector == other.mVector)
    return *this;

  mVector = other.mVector;
  emit changed();
  return *this;
}

bool OmMFInt::equals(const OmValue *other) const {
  const OmMFInt *that = dynamic_cast<const OmMFInt *>(other);
  return that && *this == *that;
}

void OmMFInt::copyFrom(const OmValue *other) {
  const OmMFInt *that = dynamic_cast<const OmMFInt *>(other);
  *this = *that;
}

bool OmMFInt::smallSeparator(int i) const {
  return (i % 10 != 0 || OmMultipleValue::smallSeparator(i));  // 10 integers per line
}
