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

#include "OmMFVector2.hpp"
#include "OmToken.hpp"
#include "OmTokenizer.hpp"

void OmMFVector2::readAndAddItem(OmTokenizer *tokenizer, const QString &worldPath) {
  double x = tokenizer->nextToken()->toDouble();
  double y = tokenizer->nextToken()->toDouble();
  mVector.append(OmVector2(x, y));
}

void OmMFVector2::clear() {
  if (!mVector.empty()) {
    mVector.clear();
    emit changed();
    emit cleared();  // notify that all children have been removed
  }
}

void OmMFVector2::insertDefaultItem(int index) {
  assert(index >= 0 && index <= size());
  mVector.insert(mVector.begin() + index, defaultNewVariant().toVector2());
  emit itemInserted(index);
  emit changed();
}

void OmMFVector2::removeItem(int index) {
  assert(index >= 0 && index < size());
  mVector.erase(mVector.begin() + index);
  emit itemRemoved(index);
  emit changed();
}

void OmMFVector2::setItem(int index, const OmVector2 &vec) {
  assert(index >= 0 && index < size());
  if (mVector[index] != vec) {
    mVector[index] = vec;
    emit itemChanged(index);
    emit changed();
  }
}

void OmMFVector2::addItem(const OmVector2 &vec) {
  mVector.append(vec);
  emit itemInserted(mVector.size() - 1);
  emit changed();
}

void OmMFVector2::insertItem(int index, const OmVector2 &vec) {
  assert(index >= 0 && index <= size());
  mVector.insert(mVector.begin() + index, vec);
  emit itemInserted(index);
  emit changed();
}

void OmMFVector2::mult(double factor) {
  for (int i = 0, size = mVector.size(); i < size; ++i)
    mVector[i] *= factor;
}

OmMFVector2 &OmMFVector2::operator=(const OmMFVector2 &other) {
  if (mVector == other.mVector)
    return *this;

  mVector = other.mVector;
  emit changed();
  return *this;
}

bool OmMFVector2::equals(const OmValue *other) const {
  const OmMFVector2 *that = dynamic_cast<const OmMFVector2 *>(other);
  return that && *this == *that;
}

void OmMFVector2::copyFrom(const OmValue *other) {
  const OmMFVector2 *that = dynamic_cast<const OmMFVector2 *>(other);
  *this = *that;
}
