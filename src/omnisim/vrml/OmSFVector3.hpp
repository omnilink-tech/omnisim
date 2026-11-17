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

#ifndef OM_SF_VECTOR3_HPP
#define OM_SF_VECTOR3_HPP

//
// Description: field value that contains a single OmVector3
//

#include "OmSingleValue.hpp"
#include "OmVector3.hpp"
#include "OmWriter.hpp"

class OmSFVector3 : public OmSingleValue {
  Q_OBJECT

public:
  OmSFVector3() {}
  OmSFVector3(OmTokenizer *tokenizer, const QString &worldPath) { readSFVector3(tokenizer, worldPath); }
  OmSFVector3(const OmSFVector3 &other) : mValue(other.mValue) {}
  explicit OmSFVector3(const OmVector3 &v) : mValue(v) {}
  virtual ~OmSFVector3() override {}
  void read(OmTokenizer *tokenizer, const QString &worldPath) override { readSFVector3(tokenizer, worldPath); };
  void write(OmWriter &writer) const override {
    writer << toString(writer.isOmniSim() ? OmPrecision::DOUBLE_MAX : OmPrecision::FLOAT_MAX);
  }
  OmValue *clone() const override { return new OmSFVector3(*this); }
  bool equals(const OmValue *other) const override;
  void copyFrom(const OmValue *other) override;
  OmVariant variantValue() const override { return OmVariant(mValue); }
  WbFieldType type() const override { return WB_SF_VEC3F; }
  const OmVector3 &value() const { return mValue; }
  double x() const { return mValue.x(); }
  double y() const { return mValue.y(); }
  double z() const { return mValue.z(); }
  bool isNull() const { return mValue.isNull(); }
  void setValue(const OmVector3 &v);
  void setValue(double x, double y, double z);
  void setValueNoSignal(double x, double y, double z) { mValue.setXyz(x, y, z); }
  void setValueFromOde(double x, double y, double z) {
    mValue.setXyz(x, y, z);
    emit changedByOde();
  }
  void setValueFromOde(const OmVector3 &v) {
    mValue.setXyz(v.x(), v.y(), v.z());
    emit changedByOde();
  }
  void setValueByUser(const OmVector3 &v, bool changedFromSupervisor);
  void setValue(const double xyz[3]);
  void setX(double x) { setComponent(0, x); }
  void setY(double y) { setComponent(1, y); }
  void setYnoSignal(double y) { mValue.setY(y); }
  void setZ(double z) { setComponent(2, z); }
  double component(int index) const { return mValue[index]; }
  void setComponent(int index, double d);
  void mult(double factor);
  OmSFVector3 &operator=(const OmSFVector3 &other);
  bool operator==(const OmSFVector3 &other) const { return mValue == other.mValue; }

private:
  OmVector3 mValue;
  void readSFVector3(OmTokenizer *tokenizer, const QString &worldPath);
};

#endif
