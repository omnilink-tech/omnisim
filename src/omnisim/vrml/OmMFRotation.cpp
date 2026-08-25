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

#include "OmMFRotation.hpp"
#include "OmToken.hpp"
#include "OmTokenizer.hpp"

void OmMFRotation::readAndAddItem(OmTokenizer *tokenizer, const QString &worldPath) {
  double x = tokenizer->nextToken()->toDouble();
  double y = tokenizer->nextToken()->toDouble();
  double z = tokenizer->nextToken()->toDouble();
  double a = tokenizer->nextToken()->toDouble();
  mVector.append(OmRotation(x, y, z, a));
}

void OmMFRotation::clear() {
  if (!mVector.empty()) {
    mVector.clear();
    emit changed();
    emit cleared();  // notify that all children have been removed
  }
}

void OmMFRotation::insertDefaultItem(int index) {
  assert(index >= 0 && index <= size());
  mVector.insert(mVector.begin() + index, defaultNewVariant().toRotation());
  emit itemInserted(index);
  emit changed();
}

void OmMFRotation::removeItem(int index) {
  assert(index >= 0 && index < size());
  mVector.erase(mVector.begin() + index);
  emit itemRemoved(index);
  emit changed();
}

void OmMFRotation::setItem(int index, const OmRotation &rot) {
  assert(index >= 0 && index < size());
  if (mVector[index] != rot) {
    mVector[index] = rot;
    emit itemChanged(index);
    emit changed();
  }
}

void OmMFRotation::addItem(const OmRotation &rot) {
  mVector.append(rot);
  emit itemInserted(mVector.size() - 1);
  emit changed();
}

void OmMFRotation::insertItem(int index, const OmRotation &rot) {
  assert(index >= 0 && index <= size());
  mVector.insert(mVector.begin() + index, rot);
  emit itemInserted(index);
  emit changed();
}

OmMFRotation &OmMFRotation::operator=(const OmMFRotation &other) {
  if (mVector == other.mVector)
    return *this;

  mVector = other.mVector;
  emit changed();
  return *this;
}

bool OmMFRotation::equals(const OmValue *other) const {
  const OmMFRotation *that = dynamic_cast<const OmMFRotation *>(other);
  return that && *this == *that;
}

void OmMFRotation::copyFrom(const OmValue *other) {
  const OmMFRotation *that = dynamic_cast<const OmMFRotation *>(other);
  *this = *that;
}
