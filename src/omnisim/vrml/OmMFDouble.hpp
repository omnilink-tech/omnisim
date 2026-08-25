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

#ifndef OM_MF_DOUBLE_HPP
#define OM_MF_DOUBLE_HPP

//
// Description: field value that contains a multiple doubles
//

#include "OmMultipleValue.hpp"

#include "OmPrecision.hpp"
#include "OmWriter.hpp"

#include <QtCore/QVector>

#include <cassert>

class OmMFDouble : public OmMultipleValue {
  Q_OBJECT

public:
  typedef OmMFIterator<OmMFDouble, double> Iterator;

  OmMFDouble(OmTokenizer *tokenizer, const QString &worldPath) { read(tokenizer, worldPath); }
  OmMFDouble(const OmMFDouble &other) : mVector(other.mVector) {}
  virtual ~OmMFDouble() override {}
  OmValue *clone() const override { return new OmMFDouble(*this); }
  bool equals(const OmValue *other) const override;
  void copyFrom(const OmValue *other) override;
  int size() const override { return mVector.size(); }
  void clear() override;
  void writeItem(OmWriter &writer, int index) const override {
    assert(index >= 0 && index < size());
    writer << itemToString(index, writer.isOmniSim() ? OmPrecision::DOUBLE_MAX : OmPrecision::FLOAT_MAX);
  }
  void insertDefaultItem(int index) override;
  OmVariant defaultNewVariant() const override { return OmVariant(0.0); }
  void removeItem(int index) override;
  OmVariant variantValue(int index) const override {
    assert(index >= 0 && index < size());
    return OmVariant(mVector[index]);
  }
  WbFieldType type() const override { return WB_MF_FLOAT; }
  const double &item(int index) const {
    assert(index >= 0 && index < size());
    return mVector[index];
  }
  void copyItemsTo(double values[], int max = -1) const;
  void findMinMax(double *min, double *max) const;
  void setItem(int index, double value);
  void setAllItems(const double *values);
  void multiplyAllItems(double factor);
  void addItem(double value);
  void insertItem(int index, double value);
  OmMFDouble &operator=(const OmMFDouble &other);
  bool operator==(const OmMFDouble &other) const { return mVector == other.mVector; }

protected:
  void readAndAddItem(OmTokenizer *tokenizer, const QString &worldPath) override;
  bool smallSeparator(int i) const override;

private:
  QVector<double> mVector;
};

#endif
