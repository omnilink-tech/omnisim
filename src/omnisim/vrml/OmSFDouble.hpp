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

#ifndef OM_SF_DOUBLE_HPP
#define OM_SF_DOUBLE_HPP

//
// Description: field value that contains a single double
//

#include "OmSingleValue.hpp"
#include "OmWriter.hpp"

#include "OmPrecision.hpp"

class OmSFDouble : public OmSingleValue {
  Q_OBJECT

public:
  OmSFDouble(OmTokenizer *tokenizer, const QString &worldPath) { readSFDouble(tokenizer, worldPath); }
  OmSFDouble(const OmSFDouble &other) : mValue(other.mValue) {}
  explicit OmSFDouble(double d) : mValue(d) {}
  virtual ~OmSFDouble() override {}
  void read(OmTokenizer *tokenizer, const QString &worldPath) override { readSFDouble(tokenizer, worldPath); }
  void write(OmWriter &writer) const override {
    writer << toString(writer.isOmniSim() ? OmPrecision::DOUBLE_MAX : OmPrecision::FLOAT_MAX);
  }
  OmValue *clone() const override { return new OmSFDouble(*this); }
  bool equals(const OmValue *other) const override;
  void copyFrom(const OmValue *other) override;
  OmVariant variantValue() const override { return OmVariant(mValue); }
  WbFieldType type() const override { return WB_SF_FLOAT; }
  double value() const { return mValue; }
  const double *valuePointer() const { return &mValue; }
  bool isZero() const { return mValue == 0.0; }
  void setValue(double d);
  void setValueNoSignal(double d) { mValue = d; }
  void setValueFromOde(double d) {
    mValue = d;
    emit changedByOde();
  }
  void add(double d);
  void mult(double factor);
  void makeAbsolute();                // absolute value
  bool clip(double min, double max);  // clip between min and max, return true if the value was changed
  OmSFDouble &operator=(const OmSFDouble &other);
  bool operator==(const OmSFDouble &other) const { return mValue == other.mValue; }

private:
  double mValue;
  void readSFDouble(OmTokenizer *tokenizer, const QString &worldPath);
};

#endif
