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

#ifndef OM_SF_INT_HPP
#define OM_SF_INT_HPP

//
// Description: field value that contains a single integer
//

#include "OmSingleValue.hpp"
#include "OmWriter.hpp"

class OmSFInt : public OmSingleValue {
  Q_OBJECT

public:
  OmSFInt(OmTokenizer *tokenizer, const QString &worldPath) { readSFInt(tokenizer, worldPath); }
  OmSFInt(const OmSFInt &other);
  explicit OmSFInt(int value) : mValue(value) {}
  virtual ~OmSFInt() override {}
  void read(OmTokenizer *tokenizer, const QString &worldPath) override { readSFInt(tokenizer, worldPath); }
  void write(OmWriter &writer) const override { writer << mValue; }
  OmValue *clone() const override { return new OmSFInt(*this); }
  bool equals(const OmValue *other) const override;
  void copyFrom(const OmValue *other) override;
  OmVariant variantValue() const override { return OmVariant(mValue); }
  WbFieldType type() const override { return WB_SF_INT32; }
  int value() const { return mValue; }
  const int *valuePointer() const { return &mValue; }
  bool isZero() const { return mValue == 0; }
  void setValue(int i);
  void setValueNoSignal(int i) { mValue = i; }
  OmSFInt &operator=(const OmSFInt &other);
  bool operator==(const OmSFInt &other) const { return mValue == other.mValue; }

private:
  int mValue;
  void readSFInt(OmTokenizer *tokenizer, const QString &worldPath);
};

#endif
