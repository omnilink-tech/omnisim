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

#ifndef OM_MF_COLOR_HPP
#define OM_MF_COLOR_HPP

//
// Description: field value that contains a multiple OmRgb
//

#include "OmMultipleValue.hpp"
#include "OmRgb.hpp"
#include "OmWriter.hpp"

#include <QtCore/QVector>

#include <cassert>

class OmMFColor : public OmMultipleValue {
  Q_OBJECT

public:
  typedef OmMFIterator<OmMFColor, OmRgb> Iterator;

  OmMFColor(OmTokenizer *tokenizer, const QString &worldPath) { read(tokenizer, worldPath); }
  OmMFColor(const OmMFColor &other) : mVector(other.mVector) {}
  virtual ~OmMFColor() override {}
  OmValue *clone() const override { return new OmMFColor(*this); }
  bool equals(const OmValue *other) const override;
  void copyFrom(const OmValue *other) override;
  int size() const override { return mVector.size(); }
  void clear() override;
  void writeItem(OmWriter &writer, int index) const override {
    assert(index >= 0 && index < size());
    writer << itemToString(index, writer.isOmniSim() ? OmPrecision::DOUBLE_MAX : OmPrecision::FLOAT_MAX);
  }
  void insertDefaultItem(int index) override;
  OmVariant defaultNewVariant() const override { return OmVariant(OmRgb()); }
  void removeItem(int index) override;
  OmVariant variantValue(int index) const override {
    assert(index >= 0 && index < size());
    return OmVariant(mVector[index]);
  }
  WbFieldType type() const override { return WB_MF_COLOR; }
  const OmRgb &item(int index) const {
    assert(index >= 0 && index < size());
    return mVector[index];
  }
  void setItem(int index, const OmRgb &value, bool signal = true);
  void addItem(const OmRgb &value);
  void insertItem(int index, const OmRgb &value);
  OmMFColor &operator=(const OmMFColor &other);
  bool operator==(const OmMFColor &other) const { return mVector == other.mVector; }
  bool operator!=(const OmMFColor &other) const { return mVector != other.mVector; }

protected:
  void readAndAddItem(OmTokenizer *tokenizer, const QString &worldPath) override;

private:
  QVector<OmRgb> mVector;
};

#endif
