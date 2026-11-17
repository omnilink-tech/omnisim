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

#include "OmTextFind.hpp"

#include <QtGui/QTextCursor>
#include <QtGui/QTextDocument>
#include <QtWidgets/QPlainTextEdit>

QStringList OmTextFind::cFindStringList;
QStringList OmTextFind::cReplaceStringList;
OmTextFind::FindFlags OmTextFind::cLastFindFlags;
bool OmTextFind::cIsLastStringEmpty = true;

OmTextFind::OmTextFind(QPlainTextEdit *editor) : mEditor(editor) {
}

void OmTextFind::addStringToList(const QString &s, QStringList &list) {
  if (s.isEmpty())
    return;

  // prepend avoiding duplicates
  int index = list.indexOf(s);
  if (index != -1)
    list.removeAt(index);
  list.prepend(s);

  if (list.size() > 10)
    list.removeLast();
}

bool OmTextFind::find(const QString &text, FindFlags flags, bool backwards) {
  return findText(text, mEditor->textCursor().position(), flags, backwards);
}

bool OmTextFind::findFromBegin(const QString &text, FindFlags flags, bool backwards) {
  return findText(text, 0, flags, backwards);
}

bool OmTextFind::findFromEnd(const QString &text, FindFlags flags, bool backwards) {
  return findText(text, mEditor->document()->characterCount(), flags, backwards);
}

bool OmTextFind::findText(const QString &text, int position, FindFlags flags, bool backwards) {
  if (mEditor == NULL)
    return true;
  if (text.isEmpty()) {
    emit findStringChanged(QRegularExpression());
    cIsLastStringEmpty = true;
    return true;
  }

  QTextDocument *document = mEditor->document();
  QTextDocument::FindFlags findFlags;
  if (backwards)
    findFlags |= QTextDocument::FindBackward;
  if (flags & FIND_CASE_SENSITIVE)
    findFlags |= QTextDocument::FindCaseSensitively;
  if (flags & FIND_WHOLE_WORDS)
    findFlags |= QTextDocument::FindWholeWords;

  QRegularExpression regularExpression = computeRegularExpression(text, flags);
  emit findStringChanged(regularExpression);
  if (cIsLastStringEmpty || text != cFindStringList.first()) {
    addStringToList(text, cFindStringList);
    cIsLastStringEmpty = false;
  }
  cLastFindFlags = flags;

  if (backwards)
    position -= text.length();

  QTextCursor result = document->find(regularExpression, position, findFlags);
  if (result.isNull())
    return false;

  mEditor->blockSignals(true);
  mEditor->setTextCursor(result);
  mEditor->ensureCursorVisible();
  mEditor->blockSignals(false);
  return true;
}

void OmTextFind::replace(const QString &before, const QString &after, FindFlags findFlags) {
  if (mEditor == NULL)
    return;
  if (before.isEmpty()) {
    cIsLastStringEmpty = true;
    emit findStringChanged(QRegularExpression());
    return;
  }

  addStringToList(before, cFindStringList);
  addStringToList(after, cReplaceStringList);

  QTextCursor cursor = mEditor->textCursor();
  QRegularExpression beforeRegularExpression = computeRegularExpression(before, findFlags);
  QRegularExpressionMatch match = beforeRegularExpression.match(cursor.selectedText());
  if (match.hasMatch()) {
    QString realAfter;
    if (findFlags & FIND_REGULAR_EXPRESSION)
      realAfter = expandRegularExpressionReplacement(after, match.capturedTexts());
    else
      realAfter = after;
    cursor.insertText(realAfter);
    mEditor->setTextCursor(cursor);
  }
}

void OmTextFind::replaceAll(const QString &before, const QString &after, FindFlags findFlags) {
  if (mEditor == NULL)
    return;
  if (before.isEmpty()) {
    emit findStringChanged(QRegularExpression());
    return;
  }

  addStringToList(before, cFindStringList);
  addStringToList(after, cReplaceStringList);

  QTextCursor cursor = mEditor->textCursor();
  cursor.movePosition(QTextCursor::Start);
  cursor.beginEditBlock();

  QTextDocument::FindFlags docFindFlags;
  if (findFlags & FIND_CASE_SENSITIVE)
    docFindFlags |= QTextDocument::FindCaseSensitively;
  if (findFlags & FIND_WHOLE_WORDS)
    docFindFlags |= QTextDocument::FindWholeWords;
  QRegularExpression beforeRegularExpression = computeRegularExpression(before, findFlags);

  QTextDocument *doc = mEditor->document();
  QTextCursor found = doc->find(beforeRegularExpression, cursor, docFindFlags);
  bool first = true;
  while (!found.isNull()) {
    if (found == cursor && !first) {
      if (cursor.atEnd())
        break;

      // avoid endless loop of regular expressions
      QTextCursor newPosCursor = cursor;
      newPosCursor.movePosition(QTextCursor::NextCharacter);
      found = doc->find(beforeRegularExpression, newPosCursor, docFindFlags);
    }

    first = false;

    cursor.setPosition(found.selectionStart());
    cursor.setPosition(found.selectionEnd(), QTextCursor::KeepAnchor);
    const QRegularExpressionMatch match = beforeRegularExpression.match(found.selectedText());

    QString realAfter;
    if (findFlags & FIND_REGULAR_EXPRESSION)
      realAfter = expandRegularExpressionReplacement(after, match.capturedTexts());
    else
      realAfter = after;
    cursor.insertText(realAfter);
    found = doc->find(beforeRegularExpression, cursor, docFindFlags);
  }

  cursor.endEditBlock();
}

QString OmTextFind::expandRegularExpressionReplacement(const QString &replaceText, const QStringList &capturedTexts) {
  // handles \1 \\ \& & \t \n
  QString result;
  const int numCaptures = capturedTexts.size() - 1;
  for (int i = 0; i < replaceText.length(); ++i) {
    QChar c = replaceText.at(i);
    if (c == QLatin1Char('\\') && i < replaceText.length() - 1) {
      c = replaceText.at(++i);
      if (c == QLatin1Char('\\'))
        result += QLatin1Char('\\');
      else if (c == QLatin1Char('&'))
        result += QLatin1Char('&');
      else if (c == QLatin1Char('t'))
        result += QLatin1Char('\t');
      else if (c == QLatin1Char('n'))
        result += QLatin1Char('\n');
      else if (c.isDigit()) {
        int index = c.unicode() - '1';
        if (index < numCaptures)
          result += capturedTexts.at(index + 1);
        else {
          result += QLatin1Char('\\');
          result += c;
        }
      } else {
        result += QLatin1Char('\\');
        result += c;
      }
    } else if (c == QLatin1Char('&'))
      result += capturedTexts.at(0);
    else
      result += c;
  }
  return result;
}

QRegularExpression OmTextFind::computeRegularExpression(const QString &pattern, FindFlags flags) {
  QString expPattern = pattern;
  if ((flags & FIND_REGULAR_EXPRESSION) == 0)
    expPattern = QRegularExpression::escape(pattern);
  if (flags & FIND_WHOLE_WORDS)
    expPattern = "\\b" + expPattern + "\\b";
  return QRegularExpression(expPattern, (flags & FIND_CASE_SENSITIVE) ? QRegularExpression::CaseInsensitiveOption :
                                                                        QRegularExpression::NoPatternOption);
}
