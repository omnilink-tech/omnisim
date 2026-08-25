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

#ifndef OM_MF_STRING_HPP
#define OM_MF_STRING_HPP

//
// Description: field value that contains a multiple QString
//

#include "OmMultipleValue.hpp"
#include "OmWriter.hpp"

#include <QtCore/QStringList>

#include <cassert>

class OmMFString : public OmMultipleValue {
  Q_OBJECT

public:
  typedef OmMFIterator<OmMFString, QString> Iterator;

  OmMFString(OmTokenizer *tokenizer, const QString &worldPath) { read(tokenizer, worldPath); }
  OmMFString(const OmMFString &other) : mValue(other.mValue) {}
  explicit OmMFString(const QStringList &value) : mValue(value) {}
  virtual ~OmMFString() override {}
  OmValue *clone() const override { return new OmMFString(*this); }
  bool equals(const OmValue *other) const override;
  void copyFrom(const OmValue *other) override;
  int size() const override { return mValue.size(); }
  void clear() override;
  void writeItem(OmWriter &writer, int index) const override {
    assert(index >= 0 && index < size());
    writer.writeLiteralString(mValue[index]);
  }
  void insertDefaultItem(int index) override;
  OmVariant defaultNewVariant() const override { return OmVariant(QString()); }
  void removeItem(int index) override;
  OmVariant variantValue(int index) const override {
    assert(index >= 0 && index < size());
    return OmVariant(mValue[index]);
  }
  const QStringList &value() const { return mValue; }
  WbFieldType type() const override { return WB_MF_STRING; }
  const QString &item(int index) const {
    assert(index >= 0 && index < size());
    return mValue[index];
  }
  void setValue(const QStringList &value);
  void setItem(int index, const QString &value);
  void addItem(const QString &value);
  void insertItem(int index, const QString &value);
  OmMFString &operator=(const OmMFString &other);
  bool operator==(const OmMFString &other) const { return mValue == other.mValue; }

protected:
  void readAndAddItem(OmTokenizer *tokenizer, const QString &worldPath) override;
  bool smallSeparator(int i) const override { return false; }

private:
  QStringList mValue;
};

#endif
