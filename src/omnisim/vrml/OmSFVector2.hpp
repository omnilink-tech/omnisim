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

#ifndef OM_SF_VECTOR2_HPP
#define OM_SF_VECTOR2_HPP

//
// Description: field value that contains a single OmVector2
//

#include "OmSingleValue.hpp"
#include "OmVector2.hpp"
#include "OmWriter.hpp"

class OmSFVector2 : public OmSingleValue {
  Q_OBJECT

public:
  OmSFVector2(OmTokenizer *tokenizer, const QString &worldPath) { readSFVector2(tokenizer, worldPath); }
  OmSFVector2(const OmSFVector2 &other) : mValue(other.mValue) {}
  virtual ~OmSFVector2() override {}
  void read(OmTokenizer *tokenizer, const QString &worldPath) override { readSFVector2(tokenizer, worldPath); }
  void write(OmWriter &writer) const override {
    writer << toString(writer.isOmniSim() ? OmPrecision::DOUBLE_MAX : OmPrecision::FLOAT_MAX);
  }
  OmValue *clone() const override { return new OmSFVector2(*this); }
  bool equals(const OmValue *other) const override;
  void copyFrom(const OmValue *other) override;
  OmVariant variantValue() const override { return OmVariant(mValue); }
  WbFieldType type() const override { return WB_SF_VEC2F; }
  const OmVector2 &value() const { return mValue; }
  double x() const { return mValue.x(); }
  double y() const { return mValue.y(); }
  void setValue(const OmVector2 &v);
  void setValue(double x, double y);
  void setValue(const double xy[2]);
  void setValueFromOmniSim(const OmVector2 &v);
  void setX(double x) { setComponent(0, x); }
  void setY(double y) { setComponent(1, y); }
  double component(int index) const { return mValue[index]; }
  void setComponent(int index, double d);
  void mult(double factor);
  OmSFVector2 &operator=(const OmSFVector2 &other);
  bool operator==(const OmSFVector2 &other) const { return mValue == other.mValue; }

private:
  OmVector2 mValue;
  void readSFVector2(OmTokenizer *tokenizer, const QString &worldPath);
};

#endif
