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

#ifndef OM_VRML_WRITER_HPP
#define OM_VRML_WRITER_HPP

//
// Description: a text stream specialized for writing indented VRML or W3D
//

#include <QtCore/QHash>
#include <QtCore/QMap>

#include "OmVector3.hpp"

class QIODevice;

class OmNode;
class OmVector2;
class OmVector4;
class OmRotation;
class OmQuaternion;
class OmRgb;

class OmWriter {
public:
  OmWriter(QIODevice *device, const QString &fileName);
  OmWriter(QString *target, const QString &fileName);
  virtual ~OmWriter();

  bool isW3d() const { return mType == W3D; }
  bool isProto() const { return mType == PROTO; }
  bool isUrdf() const { return mType == URDF; }
  bool isOmniSim() const { return mType == VRML_SIM || mType == PROTO; }
  bool isWritingToFile() const { return mIsWritingToFile; }
  QString *string() const { return mString; };
  QString path() const;

  void writeLiteralString(const QString &string);
  void writeMFStart();
  void writeMFSeparator(bool first, bool smallSeparator);
  void writeMFEnd(bool empty);
  void writeFieldStart(const QString &name, bool w3dQuote);
  void writeFieldEnd(bool w3dQuote);

  const OmVector3 &jointOffset() const { return mJointOffset; }
  void setJointOffset(const OmVector3 &offset) { mJointOffset = offset; }

  // change current indentation
  void increaseIndent() { mIndent++; }
  void decreaseIndent() { mIndent--; }

  // write current indentation
  void indent();

  // write .wbt, .w3d or .urdf header and footer based on VrmlType
  void writeHeader(const QString &title);
  void writeFooter(const QStringList *info = NULL);

  void setRootNode(OmNode *node) { mRootNode = node; }
  OmNode *rootNode() const { return mRootNode; }
  void trackDeclaration(const QString &protoName, const QString &protoUrl) {
    mTrackedDeclarations.append(std::pair<QString, QString>(protoName, protoUrl));
  };
  const QList<std::pair<QString, QString>> &declarations() const { return mTrackedDeclarations; };

  QMap<uint64_t, QString> &indexedFaceSetDefMap() { return mIndexedFaceSetDefMap; }
  OmWriter &operator<<(const QString &s);
  OmWriter &operator<<(char);
  OmWriter &operator<<(int);
  OmWriter &operator<<(unsigned int);
  OmWriter &operator<<(float);
  OmWriter &operator<<(double);
  OmWriter &operator<<(const OmVector2 &v);
  OmWriter &operator<<(const OmVector3 &v);
  OmWriter &operator<<(const OmVector4 &v);
  OmWriter &operator<<(const OmRotation &r);
  OmWriter &operator<<(const OmQuaternion &q);
  OmWriter &operator<<(const OmRgb &rgb);

  static QString relativeTexturesPath() { return "textures/"; }
  static QString relativeMeshesPath() { return "meshes/"; }
  static QString relativeSoundsPath() { return "sounds/"; }

private:
  void setType();

  enum Type { VRML_SIM, W3D, PROTO, URDF };
  QString *mString;
  QIODevice *mDevice;
  QString mFileName;
  Type mType;
  int mIndent;
  QMap<uint64_t, QString> mIndexedFaceSetDefMap;
  bool mIsWritingToFile;
  OmVector3 mJointOffset;
  // variables used by 'convert root to basenode' writer
  OmNode *mRootNode;
  QList<std::pair<QString, QString>> mTrackedDeclarations;  // keep track of declarations that need to change level
};

#endif
