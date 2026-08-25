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

#ifndef OM_SF_STRING_HPP
#define OM_SF_STRING_HPP

//
// Description: field value that contains a single QString
//

#include "OmSingleValue.hpp"
#include "OmWriter.hpp"

class OmSFString : public OmSingleValue {
  Q_OBJECT

public:
  explicit OmSFString(const QString &s) : mValue(s) {}
  OmSFString(OmTokenizer *tokenizer, const QString &worldPath) { readSFString(tokenizer, worldPath); }
  OmSFString(const OmSFString &other) : mValue(other.mValue) {}
  virtual ~OmSFString() override {}
  void read(OmTokenizer *tokenizer, const QString &worldPath) override { readSFString(tokenizer, worldPath); }
  void write(OmWriter &writer) const override { writer.writeLiteralString(mValue); }
  OmValue *clone() const override { return new OmSFString(*this); }
  bool equals(const OmValue *other) const override;
  void copyFrom(const OmValue *other) override;
  OmVariant variantValue() const override { return OmVariant(mValue); }
  WbFieldType type() const override { return WB_SF_STRING; }
  const QString &value() const { return mValue; }
  bool isEmpty() const { return mValue.isEmpty(); }
  void clear() { return mValue.clear(); }
  void setValue(const QString &s);
  OmSFString &operator=(const OmSFString &other);
  bool operator==(const OmSFString &other) const { return mValue == other.mValue; }

private:
  QString mValue;
  void readSFString(OmTokenizer *tokenizer, const QString &worldPath);
};

#endif
