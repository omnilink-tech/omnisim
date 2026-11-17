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

#include "OmMFString.hpp"
#include "OmToken.hpp"
#include "OmTokenizer.hpp"

void OmMFString::readAndAddItem(OmTokenizer *tokenizer, const QString &worldPath) {
  mValue.append(tokenizer->nextToken()->toString());
}

void OmMFString::clear() {
  if (!mValue.empty()) {
    mValue.clear();
    emit changed();
    emit cleared();  // notify that all children have been removed
  }
}

void OmMFString::insertDefaultItem(int index) {
  assert(index >= 0 && index <= size());
  mValue.insert(mValue.begin() + index, defaultNewVariant().toString());
  emit itemInserted(index);
  emit changed();
}

void OmMFString::removeItem(int index) {
  assert(index >= 0 && index < size());
  mValue.erase(mValue.begin() + index);
  emit itemRemoved(index);
  emit changed();
}

void OmMFString::setValue(const QStringList &value) {
  mValue = value;
  emit changed();
}

void OmMFString::setItem(int index, const QString &value) {
  assert(index >= 0 && index < size());
  if (mValue[index] != value) {
    mValue[index] = value;
    emit itemChanged(index);
    emit changed();
  }
}

void OmMFString::addItem(const QString &value) {
  mValue.push_back(value);
  emit itemInserted(mValue.size() - 1);
  emit changed();
}

void OmMFString::insertItem(int index, const QString &value) {
  assert(index >= 0 && index <= size());
  mValue.insert(mValue.begin() + index, value);
  emit itemInserted(index);
  emit changed();
}

OmMFString &OmMFString::operator=(const OmMFString &other) {
  if (mValue == other.mValue)
    return *this;

  mValue = other.mValue;
  emit changed();
  return *this;
}

bool OmMFString::equals(const OmValue *other) const {
  const OmMFString *that = dynamic_cast<const OmMFString *>(other);
  return that && *this == *that;
}

void OmMFString::copyFrom(const OmValue *other) {
  const OmMFString *that = dynamic_cast<const OmMFString *>(other);
  *this = *that;
}
