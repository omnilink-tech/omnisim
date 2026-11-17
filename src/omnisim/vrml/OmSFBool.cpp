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

#include "OmSFBool.hpp"
#include "OmToken.hpp"
#include "OmTokenizer.hpp"

OmSFBool::OmSFBool(const OmSFBool &other) {
  mValue = other.mValue;
}

void OmSFBool::readSFBool(OmTokenizer *tokenizer, const QString &worldPath) {
  try {
    mValue = tokenizer->nextToken()->toBool();
  } catch (...) {
    tokenizer->reportError(tr("Expected boolean value, found %1").arg(tokenizer->lastWord()), tokenizer->lastToken());
    tokenizer->ungetToken();  // unexpected token: keep the tokenizer coherent
    throw 0;                  // report the exception
  }
}

void OmSFBool::setValue(bool b) {
  if (mValue == b)
    return;

  mValue = b;
  emit changed();
}

OmSFBool &OmSFBool::operator=(const OmSFBool &other) {
  if (mValue == other.mValue)
    return *this;

  mValue = other.mValue;
  emit changed();
  return *this;
}

bool OmSFBool::equals(const OmValue *other) const {
  const OmSFBool *that = dynamic_cast<const OmSFBool *>(other);
  return that && *this == *that;
}

void OmSFBool::copyFrom(const OmValue *other) {
  const OmSFBool *that = dynamic_cast<const OmSFBool *>(other);
  *this = *that;
}
