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

#include "OmMFVector3.hpp"
#include "OmToken.hpp"
#include "OmTokenizer.hpp"

void OmMFVector3::readAndAddItem(OmTokenizer *tokenizer, const QString &worldPath) {
  double x = tokenizer->nextToken()->toDouble();
  double y = tokenizer->nextToken()->toDouble();
  double z = tokenizer->nextToken()->toDouble();
  mVector.append(OmVector3(x, y, z));
}

void OmMFVector3::clear() {
  if (!mVector.empty()) {
    mVector.clear();
    emit changed();
    emit cleared();  // notify that all children have been removed
  }
}

void OmMFVector3::insertDefaultItem(int index) {
  assert(index >= 0 && index <= size());
  mVector.insert(mVector.begin() + index, defaultNewVariant().toVector3());
  emit itemInserted(index);
  emit changed();
}

void OmMFVector3::removeItem(int index) {
  assert(index >= 0 && index < size());
  mVector.erase(mVector.begin() + index);
  emit itemRemoved(index);
  emit changed();
}

void OmMFVector3::setItem(int index, const OmVector3 &vec) {
  assert(index >= 0 && index < size());
  if (mVector[index] != vec) {
    mVector[index] = vec;
    emit itemChanged(index);
    emit changed();
  }
}

void OmMFVector3::rescale(const OmVector3 &scale) {
  double sx = scale.x();
  double sy = scale.y();
  double sz = scale.z();
  if (sx == 1.0 && sy == 1.0 && sz == 1.0)
    return;

  const int vectorSize = mVector.size();

  for (int index = 0; index < vectorSize; index++) {
    const OmVector3 &previousValue = mVector[index];
    mVector[index].setXyz(sx * previousValue.x(), sy * previousValue.y(), sz * previousValue.z());
    emit itemChanged(index);
  }

  emit changed();
}

void OmMFVector3::rescaleAndTranslate(int coordinate, double scale, double translation) {
  if (scale == 1.0) {
    translate(coordinate, translation);
    return;
  }

  const int vectorSize = mVector.size();
  for (int index = 0; index < vectorSize; index++) {
    mVector[index][coordinate] = scale * mVector[index][coordinate] + translation;
    emit itemChanged(index);
  }

  emit changed();
}

void OmMFVector3::rescaleAndTranslate(const OmVector3 &scale, const OmVector3 &translation) {
  double sx = scale.x();
  double sy = scale.y();
  double sz = scale.z();

  if (sx == 1.0 && sy == 1.0 && sz == 1.0) {
    translate(translation);
    return;
  }

  double tx = translation.x();
  double ty = translation.y();
  double tz = translation.z();
  const int vectorSize = mVector.size();

  for (int index = 0; index < vectorSize; index++) {
    const OmVector3 &previousValue = mVector[index];
    mVector[index].setXyz(sx * previousValue.x() + tx, sy * previousValue.y() + ty, sz * previousValue.z() + tz);
    emit itemChanged(index);
  }

  emit changed();
}

void OmMFVector3::translate(int coordinate, double translation) {
  if (translation == 0.0)
    return;

  const int vectorSize = mVector.size();
  for (int index = 0; index < vectorSize; index++) {
    mVector[index][coordinate] = mVector[index][coordinate] + translation;
    emit itemChanged(index);
  }

  emit changed();
}

void OmMFVector3::translate(const OmVector3 &translation) {
  double x = translation.x();
  double y = translation.y();
  double z = translation.z();

  if (x == 0.0 && y == 0.0 && z == 0.0)
    return;

  const int vectorSize = mVector.size();

  for (int index = 0; index < vectorSize; index++) {
    const OmVector3 &previousValue = mVector[index];
    mVector[index].setXyz(previousValue.x() + x, previousValue.y() + y, previousValue.z() + z);
    emit itemChanged(index);
  }

  emit changed();
}

void OmMFVector3::addItem(const OmVector3 &vec) {
  mVector.append(vec);
  emit itemInserted(mVector.size() - 1);
  emit changed();
}

void OmMFVector3::insertItem(int index, const OmVector3 &vec) {
  assert(index >= 0 && index <= size());
  mVector.insert(mVector.begin() + index, vec);
  emit itemInserted(index);
  emit changed();
}

void OmMFVector3::mult(double factor) {
  for (int i = 0, size = mVector.size(); i < size; ++i)
    mVector[i] *= factor;
}

OmMFVector3 &OmMFVector3::operator=(const OmMFVector3 &other) {
  if (mVector == other.mVector)
    return *this;

  mVector = other.mVector;
  emit changed();
  return *this;
}

bool OmMFVector3::equals(const OmValue *other) const {
  const OmMFVector3 *that = dynamic_cast<const OmMFVector3 *>(other);
  return that && *this == *that;
}

void OmMFVector3::copyFrom(const OmValue *other) {
  const OmMFVector3 *that = dynamic_cast<const OmMFVector3 *>(other);
  *this = *that;
}
