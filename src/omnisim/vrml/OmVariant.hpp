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

#ifndef OM_VARIANT_HPP
#define OM_VARIANT_HPP

//
// Description: The OmVariant class acts like a union for OmniSim SF data types
//              Currently used to implement the Scene Tree's clipboard, but could be used in other situations
//

#include "OmPrecision.hpp"

#include <QtCore/QObject>

class OmRgb;
class OmNode;
class QString;
class OmVector2;
class OmVector3;
class OmRotation;

class OmVariant : public QObject {
  Q_OBJECT

public:
  OmVariant();
  explicit OmVariant(bool b);
  explicit OmVariant(int i);
  explicit OmVariant(double d);
  explicit OmVariant(const QString &s);
  explicit OmVariant(const OmVector2 &v);
  explicit OmVariant(const OmVector3 &v);
  explicit OmVariant(const OmRgb &c);
  explicit OmVariant(const OmRotation &r);
  explicit OmVariant(OmNode *n);
  OmVariant(const OmVariant &other);
  OmVariant &operator=(const OmVariant &other);
  bool operator==(const OmVariant &other) const;
  bool operator!=(const OmVariant &other) const;

  virtual ~OmVariant() override;

  int type() const { return mType; }
  bool isEmpty() const { return mType == -1; }
  virtual void clear();

  const QString toSimplifiedStringRepresentation(OmPrecision::Level level = OmPrecision::DOUBLE_MAX) const;

  // setters
  virtual void setBool(bool b);
  virtual void setInt(int i);
  virtual void setDouble(double d);
  virtual void setString(const QString &s);
  virtual void setVector2(const OmVector2 &v);
  virtual void setVector3(const OmVector3 &v);
  virtual void setColor(const OmRgb &c);
  virtual void setRotation(const OmRotation &r);
  virtual void setNode(OmNode *n, bool persistent = false);

  // getters
  bool toBool() const { return mBool; }
  int toInt() const { return mInt; }
  double toDouble() const { return mDouble; }
  const QString &toString() const { return *mString; }
  const OmVector2 &toVector2() const { return *mVector2; }
  const OmVector3 &toVector3() const { return *mVector3; }
  const OmRgb &toColor() const { return *mColor; }
  const OmRotation &toRotation() const { return *mRotation; }
  virtual OmNode *toNode() const { return mNode; }

protected:
  void setValue(const OmVariant &v);

protected slots:
  void clearNode();

private:
  int mType;
  bool mOwnsNode;
  union {
    bool mBool;
    int mInt;
    double mDouble;
    QString *mString;
    OmRgb *mColor;
    OmVector2 *mVector2;
    OmVector3 *mVector3;
    OmRotation *mRotation;
    OmNode *mNode;
  };
};

#endif
