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

#include "OmWriter.hpp"

#include "OmWorldFileFormat.hpp"

#include "OmApplicationInfo.hpp"
#include "OmQuaternion.hpp"
#include "OmRgb.hpp"
#include "OmRotation.hpp"
#include "OmVector2.hpp"
#include "OmVector4.hpp"
#include "OmVersion.hpp"

#include <QtCore/QFileInfo>

OmWriter::OmWriter(QIODevice *device, const QString &fileName) :
  mString(NULL),
  mDevice(device),
  mFileName(fileName),
  mIndent(0),
  mIsWritingToFile(true),
  mJointOffset(0.0, 0.0, 0.0),
  mRootNode(NULL) {
  setType();
}

OmWriter::OmWriter(QString *target, const QString &fileName) :
  mString(target),
  mDevice(NULL),
  mFileName(fileName),
  mIndent(0),
  mIsWritingToFile(false),
  mJointOffset(0.0, 0.0, 0.0),
  mRootNode(NULL) {
  setType();
}

OmWriter::~OmWriter() {
}

void OmWriter::setType() {
  if (OmWorldFileFormat::isWorldFile(mFileName))
    mType = VRML_SIM;
  else if (mFileName.endsWith(".w3d", Qt::CaseInsensitive))
    mType = W3D;
  else if (mFileName.endsWith(".proto", Qt::CaseInsensitive))
    mType = PROTO;
  else if (mFileName.endsWith(".urdf", Qt::CaseInsensitive))
    mType = URDF;
}

QString OmWriter::path() const {
  QFileInfo p(mFileName);
  return p.path();
}

void OmWriter::writeMFStart() {
  if (!isW3d() && !isUrdf()) {
    *this << "[";
    increaseIndent();
  }
}

void OmWriter::writeMFSeparator(bool first, bool smallSeparator) {
  if (!isW3d() && !isUrdf()) {
    if (smallSeparator && !first)
      *this << ", ";
    else {
      *this << "\n";
      indent();
    }
  } else if (!first && !isUrdf())  // W3D
    *this << " ";
}

void OmWriter::writeMFEnd(bool empty) {
  if (!isW3d() && !isUrdf()) {
    decreaseIndent();
    if (!empty) {
      *this << "\n";
      indent();
    }
    *this << "]";
  }
}

void OmWriter::writeFieldStart(const QString &name, bool w3dQuote) {
  if (isW3d()) {
    *this << name + "=";
    if (w3dQuote)
      *this << "\'";
  } else {
    indent();
    *this << name + " ";
  }
}

void OmWriter::writeFieldEnd(bool w3dQuote) {
  if (isW3d()) {
    if (w3dQuote)
      *this << "\'";
  } else
    *this << "\n";
}

void OmWriter::writeLiteralString(const QString &string) {
  QString text(string);
  if (isW3d()) {
    text.replace("&", "&amp;");
    text.replace("<", "&lt;");
    text.replace(">", "&gt;");
    text.replace("'", "&#39;");
  }
  text.replace("\\", "\\\\");   // replace '\' by '\\'
  text.replace("\"", "\\\"");   // replace '"' by '\"'
  *this << '"' << text << '"';  // add double quotes
}

void OmWriter::indent() {
  for (int i = 0; i < mIndent; ++i)
    *this << "  ";
}

void OmWriter::writeHeader(const QString &title) {
  switch (mType) {
    case VRML_SIM:
      *this << QString("#OMNISIM %1 utf8\n").arg(OmApplicationInfo::version().toString(false));
      return;
    case W3D:
      *this << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
      *this << "<X3D>\n";
      *this << "<head>\n";
      *this << "<meta name=\"generator\" content=\"OmniSim\" />\n";
      *this << "<meta name=\"version\" content=\"" + OmApplicationInfo::version().toString(false) + "\" />\n";
      *this << "</head>\n";
      *this << "<Scene>\n";
      return;
    case URDF:
      *this << "<?xml version=\"1.0\"?>\n";
      *this << "<robot name=\"" + title + "\" xmlns:xacro=\"http://ros.org/wiki/xacro\">\n";
      return;
    default:
      return;
  }
}

void OmWriter::writeFooter(const QStringList *info) {
  if (isW3d()) {
    *this << "</Scene>\n";
    *this << "</X3D>\n";
  } else if (isUrdf())
    *this << "</robot>\n";
}

OmWriter &OmWriter::operator<<(const QString &s) {
  if (mString)
    *mString += s;
  else
    mDevice->write(s.toUtf8());
  return *this;
}

OmWriter &OmWriter::operator<<(char c) {
  *this << QString(c);
  return *this;
}

OmWriter &OmWriter::operator<<(int i) {
  *this << QString::number(i);
  return *this;
}

OmWriter &OmWriter::operator<<(unsigned int i) {
  *this << QString::number(i);
  return *this;
}

OmWriter &OmWriter::operator<<(float f) {
  *this << OmPrecision::doubleToString(f, OmPrecision::FLOAT_MAX);
  return *this;
}

OmWriter &OmWriter::operator<<(double f) {
  *this << OmPrecision::doubleToString(f, OmPrecision::DOUBLE_MAX);
  return *this;
}

OmWriter &OmWriter::operator<<(const OmVector2 &v) {
  *this << v.toString(OmPrecision::DOUBLE_MAX);
  return *this;
}

OmWriter &OmWriter::operator<<(const OmVector3 &v) {
  *this << v.toString(OmPrecision::DOUBLE_MAX);
  return *this;
}

OmWriter &OmWriter::operator<<(const OmVector4 &v) {
  *this << v.toString(OmPrecision::DOUBLE_MAX);
  return *this;
}

OmWriter &OmWriter::operator<<(const OmRotation &r) {
  *this << r.toString(OmPrecision::DOUBLE_MAX);
  return *this;
}

OmWriter &OmWriter::operator<<(const OmQuaternion &q) {
  *this << q.toString(OmPrecision::DOUBLE_MAX);
  return *this;
}

OmWriter &OmWriter::operator<<(const OmRgb &rgb) {
  *this << rgb.toString(OmPrecision::FLOAT_MAX);
  return *this;
}
