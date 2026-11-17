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

#ifndef OM_SF_ROTATION_HPP
#define OM_SF_ROTATION_HPP

//
// Description: field value that contains a single OmRotation
//

#include "OmRotation.hpp"
#include "OmSingleValue.hpp"
#include "OmWriter.hpp"

class OmSFRotation : public OmSingleValue {
  Q_OBJECT

public:
  OmSFRotation() {}
  OmSFRotation(OmTokenizer *tokenizer, const QString &worldPath) { readSFRotation(tokenizer, worldPath); }
  OmSFRotation(const OmSFRotation &other) : mValue(other.mValue) {}
  explicit OmSFRotation(const OmRotation &r) : mValue(r) {}
  virtual ~OmSFRotation() override {}
  void read(OmTokenizer *tokenizer, const QString &worldPath) override { readSFRotation(tokenizer, worldPath); }
  void write(OmWriter &writer) const override {
    writer << toString(writer.isOmniSim() ? OmPrecision::DOUBLE_MAX : OmPrecision::FLOAT_MAX);
  }
  OmValue *clone() const override { return new OmSFRotation(*this); }
  bool equals(const OmValue *other) const override;
  void copyFrom(const OmValue *other) override;
  OmVariant variantValue() const override { return OmVariant(mValue); }
  WbFieldType type() const override { return WB_SF_ROTATION; }
  const OmRotation &value() const { return mValue; }
  double x() const { return mValue.x(); }
  double y() const { return mValue.y(); }
  double z() const { return mValue.z(); }
  double angle() const { return mValue.angle(); }
  void setValue(const OmRotation &v);
  void inline setValueFromOde(const OmRotation &v);
  void setValue(double x, double y, double z, double angle);
  void inline setValueFromOde(double x, double y, double z, double angle);
  void setValueByUser(const OmRotation &v, bool changedFromSupervisor);
  void setX(double x) { setComponent(0, x); }
  void setY(double y) { setComponent(1, y); }
  void setZ(double z) { setComponent(2, z); }
  void setAngle(double angle) { setComponent(3, angle); }
  void setComponent(int index, double d);
  double component(int index) const;
  OmSFRotation &operator=(const OmSFRotation &other);
  bool operator==(const OmSFRotation &other) const { return mValue == other.mValue; }

private:
  OmRotation mValue;
  void readSFRotation(OmTokenizer *tokenizer, const QString &worldPath);
};

void inline OmSFRotation::setValueFromOde(double x, double y, double z, double angle) {
  if (mValue == OmRotation(x, y, z, angle))
    return;

  mValue.setAxisAngle(x, y, z, angle);
  emit changedByOde();
}

void inline OmSFRotation::setValueFromOde(const OmRotation &v) {
  if (mValue == v)
    return;

  mValue = v;
  emit changedByOde();
}

#endif
