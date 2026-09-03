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

#include "OmTokenizer.hpp"
#include "OmWorldFileFormat.hpp"

#include "OmApplicationInfo.hpp"
#include "OmFileUtil.hpp"
#include "OmLog.hpp"
#include "OmNetwork.hpp"
#include "OmProtoTemplateEngine.hpp"
#include "OmStandardPaths.hpp"
#include "OmToken.hpp"
#include "OmUrdfImporter.hpp"
#include "OmUrl.hpp"

#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QRegularExpression>
#include <QtCore/QStandardPaths>
#include <QtCore/QStringList>
#include <QtCore/QTextStream>

#include <cassert>

static OmVersion cWorldFileVersion;

const OmVersion &OmTokenizer::worldFileVersion() {
  return cWorldFileVersion;
}

OmTokenizer::OmTokenizer() :
  mFileType(UNKNOWN),
  mFileVersion(OmApplicationInfo::version()),
  mStream(NULL),
  mLine(1),
  mColumn(0),
  mTokenLine(1),
  mTokenColumn(-1),
  mIndex(-1),
  mAtEnd(false),
  mErrorOffset(0) {
}

OmTokenizer::~OmTokenizer() {
  qDeleteAll(mVector);
}

void OmTokenizer::skipToken(const char *expectedWord) {
  if (!hasMoreTokens()) {
    reportError(QObject::tr("Expected '%1' but reached the end of the file").arg(expectedWord), lastToken());
    throw 0;
  }

  const OmToken *token = nextToken();

  if (token->word() != expectedWord) {
    reportError(QObject::tr("Expected '%1' but found '%2'").arg(expectedWord).arg(token->word()), token);
    throw 0;
  }
}

const QString &OmTokenizer::nextWord() {
  return nextToken()->word();
}

OmToken *OmTokenizer::lastToken() const {
  assert(mIndex > 0);
  if (mIndex > 0)
    return mVector[mIndex - 1];
  else
    return NULL;
}

const QString &OmTokenizer::lastWord() const {
  // cppcheck-suppress unassignedVariable
  // cppcheck-suppress variableScope
  static QString emptyWord;
  if (lastToken())
    return lastToken()->word();
  else
    return emptyWord;
}

const QString &OmTokenizer::peekWord() const {
  return peekToken()->word();
}

void OmTokenizer::markTokenStart() {
  mTokenLine = mLine;
  mTokenColumn = mColumn;
}

void OmTokenizer::displayHeaderHelp(const QString &fileName, const QString &headerTag) {
  const OmVersion &v = OmApplicationInfo::version();
  OmLog::info(
    QObject::tr("Please modify the first line of '%1' to \"#%2 %3 utf8\".").arg(fileName).arg(headerTag).arg(v.toString(false)),
    false, OmLog::PARSING);
}

bool OmTokenizer::readFileInfo(bool headerRequired, bool displayWarning, const QString &headerTag, bool isProto) {
  // reset version
  const OmVersion &releaseVersion = OmApplicationInfo::version();
  mFileVersion = releaseVersion;

  // store all the comments into mInfo
  while (true) {
    qint64 savedPos = mStream->pos();
    QString line = readLine();
    if (line.startsWith('#')) {
      line = line.mid(1).trimmed();  // remove '#' and whitespace at the beginning and end
      mInfo.append(line + '\n');
    } else {
      mStream->seek(savedPos);
      mLine--;        // one extra line was read
      mInfo.chop(1);  // remove last '\n'
      break;
    }
  }

  // empty info case
  if (mInfo.isEmpty()) {
    if (headerRequired) {
      OmLog::error(QObject::tr("'%1': error: Missing header.").arg(mFileName), false, OmLog::PARSING);
      displayHeaderHelp(mFileName, headerTag);
      return false;
    } else
      return true;
  }
  // get the first line
  QStringList splittedInfo = mInfo.split('\n');
  QString header = splittedInfo[0];

  // check if the first line is an header

  // matches examples:
  //   "#OMNISIM R2025a utf8"   (canonical)
  //   "#VRML_SIM R2018a utf8"  (legacy upstream form, accepted forever)
  //   "#VRML_SIM V6.0 utf8"
  // The alternation MUST stay non-capturing: OmVersion::fromString builds a
  // regex whose numbered capture groups 1-6 carry the version fields.
  bool found = mFileVersion.fromString(header, "^(?:OMNISIM|VRML_SIM) ", " utf8$");

  if (found) {
    if (mFileType == WORLD)
      cWorldFileVersion = mFileVersion;
    // remove the header and trim whitespaces from mInfo
    mInfo.clear();
    for (int i = 1; i < splittedInfo.size(); ++i)
      mInfo.append(splittedInfo[i].trimmed() + '\n');
    mInfo.chop(1);  // remove last '\n'

    if (mFileType == MODEL)
      return true;

    // do a forward compatibility test based on the file and webots versions without the maintenance id
    OmVersion forwardCompatiblityFileVersion = mFileVersion;
    forwardCompatiblityFileVersion.setRevision(0);
    OmVersion forwardCompatiblityEngineVersion = releaseVersion;
    forwardCompatiblityEngineVersion.setRevision(0);
    const OmVersion r2021b(2021, 1, 0);
    if (forwardCompatiblityFileVersion > forwardCompatiblityEngineVersion)
      OmLog::warning(QObject::tr("'%1': This file was created by OmniSim %2 while you are using OmniSim %3. "
                                 "Forward compatibility may not work.")
                       .arg(mFileName)
                       .arg(mFileVersion.toString())
                       .arg(releaseVersion.toString()),
                     false, OmLog::PARSING);
    else if (forwardCompatiblityFileVersion < r2021b && forwardCompatiblityEngineVersion >= r2021b)
      OmLog::warning(
        QObject::tr("'%1': This file was created with OmniSim %2 while you are using OmniSim %3. "
                    "You may need to adjust urls for textures and meshes, see details in the change log of OmniSim R2021b.")
          .arg(mFileName)
          .arg(mFileVersion.toString())
          .arg(releaseVersion.toString()),
        false, OmLog::PARSING);

    return true;
  } else {
    if (headerRequired) {
      OmLog::error(QObject::tr("'%1': Invalid header.").arg(mFileName), false, OmLog::PARSING);
      displayHeaderHelp(mFileName, headerTag);
      return false;
    } else {
      if (displayWarning) {
        OmLog::warning(QObject::tr("'%1': Missing header.").arg(mFileName), false, OmLog::PARSING);
        displayHeaderHelp(mFileName, headerTag);
      }
      return true;
    }
  }
}

bool OmTokenizer::checkFileHeader() {
  switch (mFileType) {
    case WORLD:
      return readFileInfo(true, true, "OMNISIM");
    case PROTO:
      return readFileInfo(true, true, "OMNISIM", true);
    case MODEL:
      return readFileInfo(false, false, "VRML");
    default:
      return true;
  }
}

QString OmTokenizer::readLine() {
  mLine++;
  mColumn = 0;
  return mStream->readLine();
}

QChar OmTokenizer::readChar() {
  if (mStream->atEnd()) {
    if (!mAtEnd) {
      mAtEnd = true;
      return '\n';
    }
    throw 0;
  }

  QChar c;
  *mStream >> c;
  mColumn++;

  if (c == '\n') {
    mLine++;
    mColumn = 0;
  }

  return c;
}

void OmTokenizer::skipWhiteSpace() {
  while (OmToken::isSpace(mChar) || mChar == '#') {
    // skip comments
    if (mChar == '#') {
      mChar = readChar();
      while (mChar != '\n')
        mChar = readChar();
    } else
      mChar = readChar();
  }
}

QString OmTokenizer::readWord() {
  skipWhiteSpace();

  QString word;
  word.append(mChar);
  markTokenStart();

  // handle string literals
  if (mChar == '"') {
    mChar = readChar();
    // we must find the closing double quotes
    while (mChar != '"') {
      if (mChar == '\\') {
        mChar = readChar();
        if (mChar == 'n')  // '\n' is allowed to create new line in SFString
          word.append('\\');
        else if (mChar != '\\' && mChar != '"')  // only allowed to escape double quotes and backslash
          reportError(QObject::tr("Invalid escaped character"), mLine, mColumn);
      }
      if (mChar == '\n') {
        reportError(QObject::tr("Unclosed string literal"), mTokenLine, mTokenColumn);
        mChar = '"';
        break;
      }
      word.append(mChar);
      mChar = readChar();
    }
    word.append(mChar);
    mChar = readChar();
    return word;
  }

  const QString &open = OmProtoTemplateEngine::openingToken();
  const QString &close = OmProtoTemplateEngine::closingToken();
  // tokenize template code but skip comments
  if (mChar == open[0]) {
    int nOpen = open.size();

    for (int i = 1; i < nOpen; ++i) {
      mChar = readChar();
      word.append(mChar);
      if (mChar != open[i]) {
        reportError(QObject::tr("Unexpected template statement opening. Expected='%1', Received='%2'").arg(open[i]).arg(mChar),
                    mTokenLine, mTokenColumn);
        return word;
      }
    }

    // skip comments:
    // * single line comment starting with '--'
    // * multi line comment between '--[[' and '--]]'
    int commentCharIndex = 0;  // count consecutive '-' characters
    bool shortComment = false;
    bool longComment = false;
    QChar stringStart = '\0';
    int finalEscapeCharactersCount = 0;
    while (!word.endsWith(close)) {
      mChar = readChar();

      // short or long comment found
      if (stringStart == 0) {  // ignore comment prefix in strings
        if (commentCharIndex == 2) {
          // sequence '--'
          shortComment = !longComment;
          if (longComment && mChar == ']') {
            mChar = readChar();
            if (mChar == ']') {  // sequence '--]]'
              longComment = false;
              mChar = readChar();
            }
          } else if (!longComment && mChar == '[') {
            mChar = readChar();
            if (mChar == '[') {  // sequence '--[['
              longComment = true;
              mChar = readChar();
            }
          }

          if (shortComment) {
            word.remove(-2, 2);  // remove '--'
            if (longComment)
              shortComment = false;
          }
          commentCharIndex = 0;
        }

        if (!shortComment && mChar == '-')
          ++commentCharIndex;
        else
          // count consecutive '-' characters only
          commentCharIndex = 0;
      }

      if (!shortComment && !longComment) {
        if (stringStart == mChar && finalEscapeCharactersCount % 2 == 0)
          stringStart = '\0';
        else if (stringStart == '\0' && (mChar == '\'' || mChar == '\"'))
          stringStart = mChar;
        if (mChar == '\\')
          finalEscapeCharactersCount += 1;
        else
          finalEscapeCharactersCount = 0;
        word.append(mChar);
      }

      if (shortComment && mChar == '\n')
        shortComment = false;
    }
  }

  // handle "[]{}"
  if (OmToken::isPunctuation(mChar)) {
    mChar = readChar();
    return word;
  }

  mChar = readChar();

  while (!OmToken::isSpace(mChar) && !OmToken::isPunctuation(mChar) && mChar != '#') {
    word.append(mChar);
    mChar = readChar();
  }

  return word;
}

int OmTokenizer::tokenize(const QString &fileName, const QString &prefix) {
  mFileName = fileName;
  mFileType = fileTypeFromFileName(fileName);
  mIndex = 0;

  QFile file(mFileName);
  if (!file.open(QIODevice::ReadOnly)) {
    OmLog::error(QObject::tr("Could not open file: '%1'.").arg(mFileName), false, OmLog::PARSING);
    return 1;
  }

  // Read file content. If a remote prefix is provided, splice it in
  // by rewriting the local "omnisim://" scheme.
  QByteArray contents = file.readAll();
  if (!prefix.isEmpty() && prefix != "omnisim://")
    contents.replace(QString("omnisim://").toUtf8(), prefix.toUtf8());

  // OmniSim extension: expand `#include "path/to/file.wbt"` directives
  // by splicing the referenced file's body inline. Done before URDF
  // expansion so an included file can carry its own URDFRobot blocks.
  // Only applied to world files (.wbt) — PROTOs and other types
  // wouldn't benefit and might collide with VRML preprocessor syntax.
  if (mFileType == WORLD && contents.contains("#include")) {
    QSet<QString> visited;
    visited.insert(QFileInfo(mFileName).canonicalFilePath());
    contents = expandWorldIncludes(contents, mFileName, visited);
  }

  // Native URDF support: expand any URDFRobot { url "..." } blocks into
  // standard Robot { ... } VRML before tokenization.
  if (contents.contains("URDFRobot"))
    contents = OmUrdfImporter::expandUrdfRobotBlocks(contents, mFileName);

  mStream = new QTextStream(contents);
  if (mStream->atEnd()) {
    OmLog::error(QObject::tr("File is empty: '%1'.").arg(mFileName), false, OmLog::PARSING);
    return 1;
  }

  // check .wbt header
  if (!checkFileHeader())
    return 1;

  int errors = 0;
  try {
    mChar = readChar();
    while (true) {
      QString word = readWord();
      // cppcheck-suppress constVariablePointer
      OmToken *token = new OmToken(word, mTokenLine, mTokenColumn);
      mVector.append(token);
      if (!token->isValid()) {
        reportError(QObject::tr("Invalid token \"%1\"").arg(token->word()), token);
        errors++;
      }
    }
  } catch (...) {
    // reached end of file
  }

  // add EOF token for parser
  mVector.append(new OmToken(mTokenLine, mTokenColumn));

  delete mStream;

  return errors;
}

// OmniSim extension: `#include "path"` preprocessor for .wbt world
// files. Splices the referenced file's body inline so authors can
// compose worlds (e.g. drop a 6-bot ORC roster into the cinematic
// forest environment without copy-pasting 671 lines).
//
// Recognised forms:
//   #include "relative/from/this/world.omniworld"
//   #include "/absolute/path/to/world.omniworld"
//   #include "omnisim://projects/.../world.omniworld"
//
// Lines starting with whitespace then `#include ` are matched anchored
// to a line boundary, so a literal `#include` inside a string would
// only collide if it's left-aligned — same trade-off the C
// preprocessor makes. Comments inside included files are preserved.
// Recursion is allowed up to 16 deep; circular includes are detected
// via the visited set and rejected with a parsing error (then the
// original line is left in place so the parser produces a readable
// failure rather than silently skipping content).
QByteArray OmTokenizer::expandWorldIncludes(const QByteArray &contents,
                                            const QString &currentFilePath,
                                            QSet<QString> &visited,
                                            int depth) {
  if (depth > 16) {
    OmLog::error(QObject::tr("#include nesting exceeds 16 levels in '%1'.")
                   .arg(currentFilePath),
                 false, OmLog::PARSING);
    return contents;
  }

  static const QRegularExpression includeRe(
    QStringLiteral("^[ \\t]*#include\\s+\"([^\"]+)\"\\s*$"),
    QRegularExpression::MultilineOption);
  // Strip the version header from included files so we don't end up with
  // multiple `#OMNISIM ...utf8` / `#VRML_SIM ...utf8` lines after splicing.
  static const QRegularExpression headerRe(
    QStringLiteral("^[ \\t]*#(?:VRML[A-Z_]*|OMNISIM)[ \\t]+\\S+[ \\t]+\\S+\\s*$"),
    QRegularExpression::MultilineOption);

  if (!contents.contains("#include"))
    return contents;

  const QString here = QFileInfo(currentFilePath).canonicalFilePath();
  const QString hereDir = QFileInfo(currentFilePath).absolutePath();

  QString text = QString::fromUtf8(contents);
  QString out;
  out.reserve(text.size());

  qsizetype cursor = 0;
  QRegularExpressionMatchIterator it = includeRe.globalMatch(text);
  while (it.hasNext()) {
    QRegularExpressionMatch m = it.next();
    // Copy unchanged content up to this directive.
    out.append(text.mid(cursor, m.capturedStart() - cursor));
    cursor = m.capturedEnd();

    QString rawPath = m.captured(1).trimmed();
    QString resolved;
    if (rawPath.startsWith("omnisim://")) {
      // resolveUrl rewrites omnisim:// to the runtime webots home.
      resolved = OmUrl::resolveUrl(rawPath);
    } else if (QDir::isAbsolutePath(rawPath)) {
      resolved = rawPath;
    } else {
      resolved = QDir(hereDir).filePath(rawPath);
    }

    QFileInfo info(resolved);
    const QString canonical = info.canonicalFilePath();
    if (canonical.isEmpty() || !info.exists()) {
      OmLog::error(QObject::tr("#include: file '%1' not found "
                                "(requested from '%2').")
                     .arg(rawPath, here),
                   false, OmLog::PARSING);
      // Re-emit the original directive so the parser fails informatively.
      out.append(m.captured(0));
      out.append('\n');
      continue;
    }

    if (visited.contains(canonical)) {
      OmLog::error(QObject::tr("#include: circular include of '%1' from '%2'.")
                     .arg(canonical, here),
                   false, OmLog::PARSING);
      out.append(QStringLiteral("# omnisim: skipped circular #include \""));
      out.append(rawPath);
      out.append(QStringLiteral("\"\n"));
      continue;
    }

    QFile included(canonical);
    if (!included.open(QIODevice::ReadOnly)) {
      OmLog::error(QObject::tr("#include: cannot open '%1'.").arg(canonical),
                   false, OmLog::PARSING);
      out.append(m.captured(0));
      out.append('\n');
      continue;
    }
    QByteArray incBytes = included.readAll();
    included.close();

    // Recurse first so nested #includes get expanded in the child's
    // scope (paths there are relative to THAT file, not us).
    QSet<QString> childVisited = visited;
    childVisited.insert(canonical);
    incBytes = expandWorldIncludes(incBytes, canonical, childVisited,
                                   depth + 1);

    // Strip the VRML header from the spliced content.
    QString incText = QString::fromUtf8(incBytes);
    incText.replace(headerRe, QStringLiteral(""));
    // Strip WorldInfo and Viewpoint singletons from the spliced
    // content — the top-level world owns those, and any duplicate
    // declaration would be a parser error. We do a simple brace-
    // depth scan starting at "WorldInfo {" / "Viewpoint {" at the
    // beginning of a logical line.
    auto stripSingleton = [](QString &text, const char *name) {
      const QString needle = QString::fromLatin1(name);
      int pos = 0;
      while (pos < text.length()) {
        int found = text.indexOf(needle, pos);
        if (found < 0)
          break;
        // Must be at the start of a logical line.
        int lineStart = found;
        while (lineStart > 0
               && (text[lineStart - 1] == ' '
                   || text[lineStart - 1] == '\t'))
          lineStart--;
        if (lineStart > 0 && text[lineStart - 1] != '\n') {
          pos = found + needle.length();
          continue;
        }
        // The next non-whitespace character after the name should be
        // an opening brace.
        int afterName = found + needle.length();
        while (afterName < text.length()
               && (text[afterName] == ' '
                   || text[afterName] == '\t'
                   || text[afterName] == '\n'
                   || text[afterName] == '\r'))
          afterName++;
        if (afterName >= text.length() || text[afterName] != '{') {
          pos = found + needle.length();
          continue;
        }
        int depth = 1;
        int i = afterName + 1;
        while (i < text.length() && depth > 0) {
          if (text[i] == '{')
            depth++;
          else if (text[i] == '}')
            depth--;
          i++;
        }
        if (depth != 0)
          break;  // malformed; bail out
        const QString marker = QStringLiteral("# omnisim: stripped ")
                               + needle
                               + QStringLiteral(" from include\n");
        text.replace(lineStart, i - lineStart, marker);
        pos = lineStart + marker.length();
      }
    };
    stripSingleton(incText, "WorldInfo");
    stripSingleton(incText, "Viewpoint");
    // Trim leading blank lines that the stripped header leaves behind.
    while (incText.startsWith('\n'))
      incText.remove(0, 1);

    out.append(QStringLiteral("\n# >>> begin omnisim #include \""));
    out.append(rawPath);
    out.append(QStringLiteral("\" (from "));
    out.append(QFileInfo(canonical).fileName());
    out.append(QStringLiteral(")\n"));
    out.append(incText);
    if (!incText.endsWith('\n'))
      out.append('\n');
    out.append(QStringLiteral("# <<< end omnisim #include \""));
    out.append(rawPath);
    out.append(QStringLiteral("\"\n"));
  }
  out.append(text.mid(cursor));

  return out.toUtf8();
}

void OmTokenizer::adoptTokens(const QVector<OmToken *> &tokens) {
  mIndex = 0;
  mVector.reserve(mVector.size() + tokens.size());
  foreach (const OmToken *token, tokens)
    mVector.append(new OmToken(*token));
}

int OmTokenizer::tokenizeString(const QString &string) {
  mIndex = 0;

  mStream = new QTextStream(string.toUtf8());
  if (mStream->atEnd()) {
    OmLog::error(QObject::tr("File is empty: '%1'.").arg(mFileName), false, OmLog::PARSING);
    return 1;
  }

  int errors = 0;
  try {
    mChar = readChar();
    while (true) {
      QString word = readWord();
      // cppcheck-suppress constVariablePointer
      OmToken *token = new OmToken(word, mTokenLine, mTokenColumn);
      mVector.append(token);
      if (!token->isValid()) {
        reportError(QObject::tr("Invalid token \"%1\"").arg(token->word()), token);
        errors++;
      }
    }
  } catch (...) {
    // reached end of file
  }

  // add EOF token for parser
  mVector.append(new OmToken(mTokenLine, mTokenColumn));

  delete mStream;

  return errors;
}

const QStringList OmTokenizer::tags() const {
  const QStringList lines = mInfo.split("\n");
  foreach (QString line, lines) {
    line.replace(" ", "");
    if (line.startsWith("tags:")) {
      line.remove("tags:");
      return line.split(",");
    }
  }
  return QStringList();
}

const QString OmTokenizer::license() const {
  const QStringList lines = mInfo.split("\n");
  foreach (QString line, lines) {
    if (line.startsWith("license:")) {
      line.remove("license:");
      return line;
    }
  }
  return QString();
}

const QString OmTokenizer::licenseUrl() const {
  const QStringList lines = mInfo.split("\n");
  foreach (QString line, lines) {
    if (line.startsWith("license url:")) {
      line.remove("license url:");
      return line;
    }
  }
  return QString();
}

const QString OmTokenizer::documentationUrl() const {
  const QStringList lines = mInfo.split("\n");
  foreach (QString line, lines) {
    if (line.startsWith("documentation url:")) {
      line.remove("documentation url:");
      return line.trimmed();
    }
  }
  return QString();
}

const QString OmTokenizer::parent() const {
  const QStringList lines = mInfo.split("\n");
  foreach (QString line, lines) {
    if (line.startsWith("parent:")) {
      line.remove("parent:");
      return line.trimmed();
    }
  }
  return QString();
}

void OmTokenizer::reportError(const QString &message, int line, int column) const {
  const QString prefix = mFileName.isEmpty() ? mReferralFile : mFileName;
  if (prefix.isEmpty())
    OmLog::error(QObject::tr("%1.").arg(message), false, OmLog::PARSING);
  else
    OmLog::error(QObject::tr("'%1':%2:%3: error: %4.").arg(prefix).arg(line + mErrorOffset).arg(column).arg(message), false,
                 OmLog::PARSING);
}

void OmTokenizer::reportError(const QString &message, const OmToken *token) const {
  if (!token)
    token = mVector[mIndex - 1];

  reportError(message, token->line(), token->column());
}

void OmTokenizer::reportFileError(const QString &message) const {
  const QString prefix = mFileName.isEmpty() ? mReferralFile : mFileName;
  OmLog::error(QObject::tr("'%1': error: %2.").arg(prefix, message), false, OmLog::PARSING);
}

OmTokenizer::FileType OmTokenizer::fileTypeFromFileName(const QString &fileName) {
  QString name = fileName;
  if (OmFileUtil::isLocatedInDirectory(fileName, OmStandardPaths::cachedAssetsPath())) {
    // attempting to tokenize a cached file, determine its original format from the ephemeral cache representation
    name = OmNetwork::instance()->getUrlFromEphemeralCache(fileName);
  }

  if (OmWorldFileFormat::isWorldFile(name))
    return WORLD;
  else if (name.endsWith(".proto", Qt::CaseInsensitive))
    return PROTO;
  else if (name.endsWith(".wrl", Qt::CaseInsensitive))
    return MODEL;
  else
    return UNKNOWN;
}

void OmTokenizer::skipNode(bool deleteTokens) {
  int startPos = mIndex;
  if (deleteTokens && peekWord() == "{")
    // delete node name
    --startPos;

  // move to next "{"
  while (hasMoreTokens() && nextWord() != "{") {
  }

  if (lastToken()->isEof()) {
    ungetToken();
    return;
  }

  // count the same number of opening and closing braces
  int counter = 1;
  while (counter > 0 && hasMoreTokens()) {
    const QString &word = nextWord();
    if (word == "{")
      counter++;
    else if (word == "}")
      counter--;
  }

  if (deleteTokens) {
    int count = mIndex - startPos;
    mVector.remove(startPos, count);
    mIndex = startPos;
  }
}

void OmTokenizer::skipField(bool deleteTokens) {
  if (!hasMoreTokens()) {
    reportError(QObject::tr("End of file reached while a token is expected"), lastToken());
    throw 0;
  }

  // skip node
  if (peekWord() == "USE" || peekWord() == "IS") {
    if (deleteTokens) {
      --mIndex;
      mVector.remove(mIndex, 3);
    } else {
      nextToken();
      nextToken();
    }
    return;
  }

  // skip node
  if (peekToken()->isIdentifier() || peekWord() == "DEF") {
    skipNode(deleteTokens);
    // remove field name
    --mIndex;
    mVector.remove(mIndex);
    return;
  }

  // skip unknown multiple value
  if (peekWord() == "[") {
    int startPos = mIndex - 1;
    nextToken();
    int counter = 1;
    do {
      const QString &word = nextWord();
      if (word == "[")
        counter++;
      else if (word == "]")
        counter--;
    } while (counter > 0);

    if (deleteTokens) {
      mVector.remove(startPos, mIndex - startPos);
      mIndex = startPos;
    }

    return;
  }

  // skip unknown single value
  int startPos = mIndex - 1;
  while (peekToken()->isNumeric() || peekToken()->isString() || peekToken()->isBoolean())
    nextToken();

  if (deleteTokens) {
    mVector.remove(startPos, mIndex - startPos);
    mIndex = startPos;
  }
}
