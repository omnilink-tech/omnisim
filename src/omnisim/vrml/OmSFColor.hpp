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

#ifndef OM_SF_COLOR_HPP
#define OM_SF_COLOR_HPP

//
// Description: field value that contains a single OmRgb
//

#include "OmRgb.hpp"
#include "OmSingleValue.hpp"
#include "OmWriter.hpp"

class OmSFColor : public OmSingleValue {
  Q_OBJECT

public:
  explicit OmSFColor(double r, double g, double b) : mValue(OmRgb(r, g, b)) {}
  OmSFColor(OmTokenizer *tokenizer, const QString &worldPath) { readSFColor(tokenizer, worldPath); }
  OmSFColor(const OmSFColor &other) : mValue(other.mValue) {}
  virtual ~OmSFColor() override {}
  void read(OmTokenizer *tokenizer, const QString &worldPath) override { readSFColor(tokenizer, worldPath); }
  void write(OmWriter &writer) const override {
    writer << toString(writer.isOmniSim() ? OmPrecision::DOUBLE_MAX : OmPrecision::FLOAT_MAX);
  }
  OmValue *clone() const override { return new OmSFColor(*this); }
  bool equals(const OmValue *other) const override;
  void copyFrom(const OmValue *other) override;
  OmVariant variantValue() const override { return OmVariant(mValue); }
  WbFieldType type() const override { return WB_SF_COLOR; }
  const OmRgb &value() const { return mValue; }
  double red() const { return mValue.red(); }
  double green() const { return mValue.green(); }
  double blue() const { return mValue.blue(); }
  void setValue(const OmRgb &c);
  void setValue(double r, double g, double b);     // values between 0.0 and 1.0
  void setValue(uint8_t r, uint8_t g, uint8_t b);  // values between 0 and 255
  OmSFColor &operator=(const OmSFColor &other);
  bool operator==(const OmSFColor &other) const { return mValue == other.mValue; }

private:
  OmRgb mValue;
  void readSFColor(OmTokenizer *tokenizer, const QString &worldPath);
};

#endif
