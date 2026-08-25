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

#include "OmConsole.hpp"

#include "OmActionManager.hpp"
#include "OmClipboard.hpp"
#include "OmDockTitleBar.hpp"
#include "OmFindReplaceDialog.hpp"
#include "OmLog.hpp"
#include "OmMessageBox.hpp"
#include "OmPreferences.hpp"
#include "OmRobot.hpp"
#include "OmSyntaxHighlighter.hpp"
#include "OmTextFind.hpp"
#include "OmWorld.hpp"

#include <QtGui/QAction>
#include <QtGui/QTextBlock>
#include <QtGui/QTextDocumentFragment>

#include <QtWidgets/QInputDialog>
#include <QtWidgets/QLayout>
#include <QtWidgets/QMenu>
#include <QtWidgets/QStyle>

#include <cassert>

ConsoleEdit::ConsoleEdit(QWidget *parent) : QPlainTextEdit(parent) {
  setObjectName("ConsoleEdit");
  setContextMenuPolicy(Qt::CustomContextMenu);
  connect(this, &QPlainTextEdit::customContextMenuRequested, this, &ConsoleEdit::showCustomContextMenu);

  // overwrite selection highlight format
  // resetting the automatic format applied when searching for some text
  QPalette p = palette();
  p.setColor(QPalette::Highlight, p.color(QPalette::Highlight));
  p.setColor(QPalette::HighlightedText, p.color(QPalette::HighlightedText));
  setPalette(p);

  mSyntaxHighlighter = OmSyntaxHighlighter::createForLanguage(NULL, document());
  connect(this, &QPlainTextEdit::selectionChanged, this, &ConsoleEdit::resetSearchTextHighlighting);

  // listen to clear console keyboard shortcut
  addAction(OmActionManager::instance()->action(OmAction::CLEAR_CONSOLE));
  document()->setDefaultStyleSheet("span{\n  white-space:pre;\n}\n");
}

ConsoleEdit::~ConsoleEdit() {
  delete mSyntaxHighlighter;
}

void ConsoleEdit::copy() {
  if (textCursor().hasSelection())
    OmClipboard::instance()->setString(textCursor().selection().toPlainText());
}

void ConsoleEdit::mouseDoubleClickEvent(QMouseEvent *event) {
  if (event->button() != Qt::LeftButton)
    return;

  // find position of double-click
  QTextCursor cursor(cursorForPosition(event->pos()));

  // select line under cursor
  cursor.movePosition(QTextCursor::StartOfLine);
  cursor.movePosition(QTextCursor::EndOfLine, QTextCursor::KeepAnchor);

  // mark line
  QList<QTextEdit::ExtraSelection> selections;
  QTextEdit::ExtraSelection selection;
  selection.format.setBackground(Qt::lightGray);
  selection.cursor = cursor;
  selections.append(selection);
  setExtraSelections(selections);
}

void ConsoleEdit::updateSearchTextHighlighting(QRegularExpression regularExpression) {
  if (regularExpression.pattern().isEmpty())
    disconnect(this, &QPlainTextEdit::selectionChanged, this, &ConsoleEdit::resetSearchTextHighlighting);

  mSyntaxHighlighter->setSearchTextRule(regularExpression);

  if (!regularExpression.pattern().isEmpty())
    connect(this, &QPlainTextEdit::selectionChanged, this, &ConsoleEdit::resetSearchTextHighlighting, Qt::UniqueConnection);
}

void ConsoleEdit::keyPressEvent(QKeyEvent *event) {
  if (event->modifiers() == Qt::ControlModifier) {
    switch (event->key()) {
      case Qt::Key_A:
        selectAll();
        event->accept();
        return;
      case Qt::Key_C:
        copy();
        event->accept();
        return;
      default:
        break;
    }
  }

  QPlainTextEdit::keyPressEvent(event);
}

void ConsoleEdit::focusInEvent(QFocusEvent *event) {
  QPlainTextEdit::focusInEvent(event);

  // update application actions
  OmActionManager *actionManager = OmActionManager::instance();
  actionManager->setFocusObject(this);
  actionManager->enableTextEditActions(false, true);
  actionManager->setEnabled(OmAction::COPY, textCursor().hasSelection());
  actionManager->setEnabled(OmAction::SELECT_ALL, true);
  actionManager->setEnabled(OmAction::FIND, true);
  actionManager->setEnabled(OmAction::FIND_NEXT, true);
  actionManager->setEnabled(OmAction::FIND_PREVIOUS, true);
  actionManager->setEnabled(OmAction::CUT, false);
  actionManager->setEnabled(OmAction::PASTE, false);
  actionManager->setEnabled(OmAction::UNDO, false);
  actionManager->setEnabled(OmAction::REDO, false);
}

void ConsoleEdit::focusOutEvent(QFocusEvent *event) {
  if (OmActionManager::instance()->focusObject() == this)
    OmActionManager::instance()->setFocusObject(NULL);
}

void ConsoleEdit::handleFilterChange() {
  QAction *action = dynamic_cast<QAction *>(sender());
  assert(action);

  // disable conflicting filters
  if (action->isChecked()) {
    if (action->text() == OmLog::filterName(OmLog::ALL)) {
      // disable all the specific filters
      QMenu *menu = dynamic_cast<QMenu *>(action->parent());
      assert(menu);
      const QList<QAction *> actions = menu->actions();
      // for each action of the menu
      for (int i = 0; i < actions.size(); ++i) {
        if (actions[i]->isChecked() && actions[i] != action)
          emit filterDisabled(actions[i]->text());
      }
    } else if (action->text() == OmLog::filterName(OmLog::ALL_OMNISIM)) {
      // disable all the OmniSim filters
      foreach (const QString &filter, OmLog::omniSimFilterNames())
        emit filterDisabled(filter);
      emit filterDisabled(OmLog::filterName(OmLog::ALL));
    } else if (action->text() == OmLog::filterName(OmLog::ALL_CONTROLLERS)) {
      // disable all the controller filters
      QMenu *menu = dynamic_cast<QMenu *>(action->parent());
      assert(menu);
      const QList<QAction *> actions = menu->actions();
      // for each action of the menu
      for (int i = 0; i < actions.size(); ++i) {
        if (actions[i]->isChecked() && actions[i]->property("isControllerAction").isValid())
          emit filterDisabled(actions[i]->text());
      }
      emit filterDisabled(OmLog::filterName(OmLog::ALL));
    } else {
      emit filterDisabled(OmLog::filterName(OmLog::ALL));
      if (action->property("isControllerAction").isValid())
        emit filterDisabled(OmLog::filterName(OmLog::ALL_CONTROLLERS));
      else
        emit filterDisabled(OmLog::filterName(OmLog::ALL_OMNISIM));
    }
  }

  // perform the update
  if (action->isChecked())
    emit filterEnabled(action->text());
  else
    emit filterDisabled(action->text());
}

void ConsoleEdit::handleLevelChange() {
  QAction *action = dynamic_cast<QAction *>(sender());
  assert(action);

  // disable conflicting levels
  if (action->isChecked()) {
    if (action->text() == OmLog::filterName(OmLog::ALL)) {
      // disable all the specific levels
      QMenu *menu = dynamic_cast<QMenu *>(action->parent());
      assert(menu);
      const QList<QAction *> actions = menu->actions();
      // for each action of the menu
      for (int i = 0; i < actions.size(); ++i) {
        if (actions[i]->isChecked() && actions[i] != action)
          emit levelDisabled(actions[i]->text());
      }
    } else if (action->text() == OmLog::filterName(OmLog::ALL_OMNISIM)) {
      emit levelDisabled(OmLog::levelName(OmLog::INFO));
      emit levelDisabled(OmLog::levelName(OmLog::WARNING));
      emit levelDisabled(OmLog::levelName(OmLog::ERROR));
      emit levelDisabled(OmLog::filterName(OmLog::ALL));
    } else if (action->text() == OmLog::filterName(OmLog::ALL_CONTROLLERS)) {
      emit levelDisabled(OmLog::levelName(OmLog::STDOUT));
      emit levelDisabled(OmLog::levelName(OmLog::STDERR));
      emit levelDisabled(OmLog::filterName(OmLog::ALL));
    } else {
      emit levelDisabled(OmLog::filterName(OmLog::ALL));
      if (action->text() == OmLog::levelName(OmLog::STDOUT) || action->text() == OmLog::levelName(OmLog::STDERR))
        emit levelDisabled(OmLog::filterName(OmLog::ALL_CONTROLLERS));
      else
        emit levelDisabled(OmLog::filterName(OmLog::ALL_OMNISIM));
    }
  }

  // perform the update
  if (action->isChecked())
    emit levelEnabled(action->text());
  else
    emit levelDisabled(action->text());
}

void ConsoleEdit::addContextMenuFilterItem(const QString &name, QMenu *menu, const QString &toolTip, bool isControllerAction) {
  OmConsole *console = dynamic_cast<OmConsole *>(parentWidget());
  assert(console);
  QAction *action = new QAction(menu);
  action->setText(name);
  if (!toolTip.isEmpty())
    action->setToolTip(toolTip);
  if (isControllerAction)
    action->setProperty("isControllerAction", QVariant(true));
  action->setCheckable(true);
  action->setChecked(console->getEnabledFilters().contains(name));
  menu->addAction(action);
  connect(action, &QAction::toggled, this, &ConsoleEdit::handleFilterChange);
}

void ConsoleEdit::addContextMenuLevelItem(const QString &name, QMenu *menu, const QString &toolTip) {
  OmConsole *console = dynamic_cast<OmConsole *>(parentWidget());
  assert(console);
  QAction *action = new QAction(menu);
  action->setText(name);
  if (!toolTip.isEmpty())
    action->setToolTip(toolTip);
  action->setCheckable(true);
  action->setChecked(console->getEnabledLevels().contains(name));
  menu->addAction(action);
  connect(action, &QAction::toggled, this, &ConsoleEdit::handleLevelChange);
}

void ConsoleEdit::showCustomContextMenu(const QPoint &pt) {
  OmConsole *console = dynamic_cast<OmConsole *>(parentWidget());
  assert(console);

  QMenu *menu = createStandardContextMenu();
  menu->addAction(OmActionManager::instance()->action(OmAction::FIND));
  menu->addSeparator();

  // filters
  QMenu *filterMenu = menu->addMenu(tr("&Filter"));
  addContextMenuFilterItem(OmLog::filterName(OmLog::ALL), filterMenu, tr("Display all the logs."));
  filterMenu->addSeparator();
  addContextMenuFilterItem(OmLog::filterName(OmLog::ALL_OMNISIM), filterMenu, tr("Display all the messages from OmniSim."));
  addContextMenuFilterItem(OmLog::filterName(OmLog::PARSING), filterMenu,
                           tr("Display parsing error when editing or loading a world."));
  addContextMenuFilterItem(OmLog::filterName(OmLog::ODE), filterMenu, tr("Display error messages from ODE."));
  addContextMenuFilterItem(OmLog::filterName(OmLog::JAVASCRIPT), filterMenu,
                           tr("Display Javascript log from the robot-windows."));
  addContextMenuFilterItem(OmLog::filterName(OmLog::COMPILATION), filterMenu, tr("Output from the compilation."));
  addContextMenuFilterItem(OmLog::filterName(OmLog::OMNISIM_OTHERS), filterMenu, tr("Display all the other logs."));
  filterMenu->addSeparator();
  addContextMenuFilterItem(OmLog::filterName(OmLog::ALL_CONTROLLERS), filterMenu,
                           tr("Display all the messages from the controller(s)."));
  const OmWorld *world = OmWorld::instance();
  if (world) {
    foreach (const OmRobot *robot, world->robots())
      addContextMenuFilterItem(robot->name(), filterMenu,
                               tr("Display output from the controller of the '%1' controller.").arg(robot->name()), true);
  }

  // levels
  QMenu *levelMenu = menu->addMenu(tr("&Level"));
  addContextMenuLevelItem(OmLog::filterName(OmLog::ALL), levelMenu, tr("Display all the logs."));
  levelMenu->addSeparator();
  addContextMenuLevelItem(OmLog::filterName(OmLog::ALL_OMNISIM), levelMenu, tr("Display all the OmniSim logs."));
  addContextMenuLevelItem(OmLog::levelName(OmLog::ERROR), levelMenu, tr("Displays OmniSim errors and controller(s) stderr."));
  addContextMenuLevelItem(OmLog::levelName(OmLog::WARNING), levelMenu, tr("Displays OmniSim warnings."));
  addContextMenuLevelItem(OmLog::levelName(OmLog::INFO), levelMenu, tr("Displays OmniSim info."));
  levelMenu->addSeparator();
  addContextMenuLevelItem(OmLog::filterName(OmLog::ALL_CONTROLLERS), levelMenu, tr("Display controller(s) stdout and stderr."));
  addContextMenuLevelItem(OmLog::levelName(OmLog::STDOUT), levelMenu, tr("Display controller(s) stdout."));
  addContextMenuLevelItem(OmLog::levelName(OmLog::STDERR), levelMenu, tr("Display controller(s) stderr."));
  menu->addSeparator();

  // actions
  QAction *renameAction = new QAction(this);
  renameAction->setText(tr("Rename Console"));
  connect(renameAction, &QAction::triggered, console, &OmConsole::rename);
  QAction *clearAction = new QAction(this);
  clearAction->setText(tr("Clear Console"));
  connect(clearAction, &QAction::triggered, this, &ConsoleEdit::clear);
  menu->addAction(renameAction);
  menu->addAction(clearAction);
  menu->addAction(OmActionManager::instance()->action(OmAction::CLEAR_CONSOLE));
  menu->addAction(OmActionManager::instance()->action(OmAction::NEW_CONSOLE));

  // execution
  menu->exec(mapToGlobal(pt));

  // cleanup
  const QList<QAction *> actions = filterMenu->actions() + levelMenu->actions();
  for (int i = 0; i < actions.size(); ++i)
    delete actions[i];
  menu->removeAction(renameAction);
  menu->removeAction(clearAction);
  delete renameAction;
  delete clearAction;
  delete menu;
}

OmConsole::OmConsole(QWidget *parent, const QString &name) :
  OmDockWidget(parent),
  mEnabledFilters(OmLog::filterName(OmLog::ALL)),
  mEnabledLevels(OmLog::filterName(OmLog::ALL)),
  mEditor(new ConsoleEdit(this)),
  mErrorPatterns(createErrorMatchingPatterns()),  // patterns for error matching
  mConsoleName(name),
  mBold(false),
  mUnderline(false),
  mIsOverwriteEnabled(false),  // option to overwrite last line
  mFindDialog(NULL),
  mTextFind(new OmTextFind(mEditor)) {
  updateTitle();

  titleBarWidget()->setObjectName("consoleTitleBar");
  titleBarWidget()->style()->polish(titleBarWidget());

  // create text editor
  mEditor->setReadOnly(true);
  mEditor->setMaximumBlockCount(5000);  // limit the memory usage
  mEditor->setFocusPolicy(Qt::ClickFocus);
  setWidget(mEditor);

  connect(mEditor, &ConsoleEdit::filterEnabled, this, &OmConsole::enableFilter);
  connect(mEditor, &ConsoleEdit::filterDisabled, this, &OmConsole::disableFilter);
  connect(mEditor, &ConsoleEdit::levelEnabled, this, &OmConsole::enableLevel);
  connect(mEditor, &ConsoleEdit::levelDisabled, this, &OmConsole::disableLevel);

  connect(mEditor, &ConsoleEdit::copyAvailable, this, &OmConsole::enableCopyAction);
  connect(OmActionManager::instance(), &OmActionManager::userConsoleEditCommandReceived, this, &OmConsole::handleUserCommand);

  connect(OmPreferences::instance(), &OmPreferences::changedByUser, this, &OmConsole::updateFont);
  updateFont();

  connect(OmActionManager::instance()->action(OmAction::CLEAR_CONSOLE), &QAction::triggered, this, &OmConsole::clear);

  connect(mTextFind, &OmTextFind::findStringChanged, mEditor, &ConsoleEdit::updateSearchTextHighlighting);

  // listen to OmLog
  connect(OmLog::instance(), SIGNAL(logEmitted(OmLog::Level, const QString &, bool, const QString &)), this,
          SLOT(appendLog(OmLog::Level, const QString &, bool, const QString &)));
}

void OmConsole::setEnabledFilters(const QStringList &filters) {
  mEnabledFilters = filters;
  updateTitle();
}

void OmConsole::setEnabledLevels(const QStringList &levels) {
  mEnabledLevels = levels;
  updateTitle();
}

void OmConsole::clear(bool reset) {
  mEditor->clear();
  if (reset)
    resetFormat();
}

void OmConsole::rename() {
  bool ok = false;
  const QString nameString =
    QInputDialog::getText(this, tr("Console Name"), tr("New name:"), QLineEdit::Normal, mConsoleName, &ok);
  if (ok && !nameString.isEmpty()) {
    mConsoleName = nameString;
    updateTitle();
  }
}

void OmConsole::resetFormat() {
  mForegroundColor.clear();
  mBackgroundColor.clear();
  mBold = false;
  mUnderline = false;
}

QString OmConsole::htmlSpan(const QString &s, OmLog::Level level) const {
  if (s.isEmpty() || s == "\n")
    return "";

  bool bold;
  QString foregroundColor;
  if (level == OmLog::ERROR || level == OmLog::FATAL || level == OmLog::STDERR) {
    foregroundColor = errorColor();
    bold = true;
  } else if (level == OmLog::WARNING || level == OmLog::DEBUG) {
    foregroundColor = errorColor();
    bold = false;
  } else if (level == OmLog::INFO || level == OmLog::STATUS) {
    foregroundColor = infoColor();
    bold = false;
  } else {
    assert(level == OmLog::STDOUT);
    foregroundColor = mForegroundColor;
    bold = mBold;
  }

  QString span("<span");
  if (!foregroundColor.isEmpty() || !mBackgroundColor.isEmpty() || bold || mUnderline) {
    span += " style=\"";
    if (!foregroundColor.isEmpty())
      span += "color:" + foregroundColor + ";";
    if (!mBackgroundColor.isEmpty())
      span += "background-color:" + mBackgroundColor + ";";
    if (bold)
      span += "font-weight:bold;";
    if (mUnderline)
      span += "text-decoration:underline;";
    span += "\"";
  }
  span += ">" + s.toHtmlEscaped() + "</span>";
  return span;
}

void OmConsole::handleCRAndLF(const QString &msg) {
  // handle CR (\r) and LF (\n) characters
  static bool lastLineEndsWithLF = false;

  QString html(msg);
  if (html.isEmpty())
    return;

#ifdef _WIN32
  html.replace("\r\n", "\n");  // use unix new line syntax
#endif
  html.replace("\r\n", "\n");  // CR has no effect if followed by a new line

  if (html == "<span>\n</span>" && lastLineEndsWithLF) {
    mEditor->appendHtml("");  // add empty line
    lastLineEndsWithLF = true;
    mIsOverwriteEnabled = false;
    return;
  }

  if (html.startsWith("<span>\n") && !lastLineEndsWithLF) {
    html.remove(6, 1);  // remove additional empty line
    mIsOverwriteEnabled = false;
  }

  if (html.endsWith("\n</span>")) {
    html.remove(-8, 1);  // text printed automatically on a new line
    lastLineEndsWithLF = true;
  } else
    lastLineEndsWithLF = false;

  bool lastLineEndsWithCR = false;
  if (html.endsWith("\r</span>")) {
    html.remove(-8, 1);
    lastLineEndsWithCR = !lastLineEndsWithLF;
  }
  html.remove("<span></span>");

  // handle CR in the middle of the text block
  const QStringList linesList(html.split("\r"));
  const int linesCount = linesList.size();
  bool endsWithCR = (linesCount != 1);

  for (int i = 0; i < linesCount; ++i) {
    QString line(linesList.at(i));

    if (i == linesCount - 1)
      endsWithCR = lastLineEndsWithCR;

    if (!line.isEmpty()) {
      if (mIsOverwriteEnabled) {
        // move the cursor to the beginning of last line
        QTextCursor textCursor = mEditor->textCursor();
        textCursor.movePosition(QTextCursor::End, QTextCursor::MoveAnchor);
        textCursor.movePosition(QTextCursor::StartOfLine, QTextCursor::MoveAnchor);
        textCursor.movePosition(QTextCursor::End, QTextCursor::KeepAnchor);
        QString previousLine = textCursor.selectedText();
        textCursor.removeSelectedText();

        // remove span HTML tags to compute current text length
        QString plainLine(line);
        plainLine.remove("</span>");
        int spanIndex = plainLine.indexOf("<span");
        while (spanIndex != -1) {
          int length = 5;
          while (plainLine[spanIndex + length] != '>')
            ++length;
          ++length;

          plainLine.remove(spanIndex, length);
          spanIndex = plainLine.indexOf("<span");
        }
        const int plainLineSize = plainLine.size();

        const int previousLineSize = previousLine.size();
        if (previousLineSize > plainLineSize)
          // append non-overwritten characters from previous line
          line.append(previousLine.mid(plainLineSize, previousLineSize - plainLineSize));

        textCursor.insertHtml(line);

      } else
        mEditor->appendHtml(line);
      mIsOverwriteEnabled = endsWithCR;
    } else
      mIsOverwriteEnabled = mIsOverwriteEnabled || endsWithCR;
  }
}

void OmConsole::handlePossibleAnsiEscapeSequences(const QString &msg, OmLog::Level level) {
  int i = msg.indexOf("\033[");
  if (i != -1) {  // contains ANSI escape sequences
    QString html;
    if (i != 0)  // escape code is not at the beginning of the string
      html = htmlSpan(msg.mid(0, i), level);
    while (1) {
      QString sequence;
      int msgLength = msg.length();
      if (msg.at(i) == '\x1b') {
        i += 2;  // skip the "\033[" chars
        int start = i;
        while (i < msgLength && msg.at(i++) < '\x40')
          ;
        sequence += msg.mid(start, i - start);
      }

      const QStringList codes(sequence.split(";"));  // handle multiple (e.g. sequence "ESC[0;39m" )
      foreach (const QString &code, codes) {
        // the stored sequence may be "0m" or "1m", "4m", "2J", "30m", "31m", "32m", etc.
        if (code == "0m")  // reset to default
          resetFormat();
        else if (code == "1m")  // bold
          mBold = true;
        else if (code == "4m")  // underlined
          mUnderline = true;
        else if (code.startsWith("2")) {  // clear console screen
          const char c = code.toLocal8Bit().data()[1];
          if (c == 'J') {             // code == "2J"
            OmConsole::clear(false);  // perform a clear by preserving format
            html.clear();             // nothing to output since clear has been done
          }
        } else if (code.startsWith("3")) {  // foreground color change
          const char c = code.toLocal8Bit().data()[1];
          switch (c) {
            case '0':  // code == 30m
              mForegroundColor = ansiBlack();
              break;
            case '1':  // code == 31m
              mForegroundColor = ansiRed();
              break;
            case '2':  // code == 32m
              mForegroundColor = ansiGreen();
              break;
            case '3':  // etc...
              mForegroundColor = ansiYellow();
              break;
            case '4':
              mForegroundColor = ansiBlue();
              break;
            case '5':
              mForegroundColor = ansiMagenta();
              break;
            case '6':
              mForegroundColor = ansiCyan();
              break;
            case '7':
              mForegroundColor = ansiWhite();
              break;
            case '9':  // 39m - Default text color
              mForegroundColor.clear();
              break;
            default:
              break;
          }
        } else if (code.startsWith("4")) {  // background color change
          const char c = code.toLocal8Bit().data()[1];
          switch (c) {
            case '0':  // code == 40m
              mBackgroundColor = ansiBlack();
              break;
            case '1':  // code == 41m
              mBackgroundColor = ansiRed();
              break;
            case '2':  // code == 42m
              mBackgroundColor = ansiGreen();
              break;
            case '3':  // etc...
              mBackgroundColor = ansiYellow();
              break;
            case '4':
              mBackgroundColor = ansiBlue();
              break;
            case '5':
              mBackgroundColor = ansiMagenta();
              break;
            case '6':
              mBackgroundColor = ansiCyan();
              break;
            case '7':
              mBackgroundColor = ansiWhite();
              break;
            case '9':  // 49m - Default background color
              mBackgroundColor.clear();
              break;
            default:
              break;
          }
        }
      }
      int j = i;
      i = msg.indexOf("\033[", i);
      if (i == -1) {  // Previous escape code was the last one found
        const QString remains(msg.mid(j));
        html += htmlSpan(remains, level);
        handleCRAndLF(html);
        return;
      }
      if (j != i)  // Extract text contained between two escape codes if so
        html += htmlSpan(msg.mid(j, i - j), level);
    }
  }
  handleCRAndLF(htmlSpan(msg, level));
}

void OmConsole::appendLog(OmLog::Level level, const QString &message, bool popup, const QString &logName) {
  if (message.isEmpty())
    return;

  assert(!logName.isEmpty() || level == OmLog::STATUS);

  // check enabled filters
  if (!mEnabledFilters.contains(OmLog::filterName(OmLog::ALL)) && !mEnabledFilters.contains(logName)) {
    if (OmLog::omniSimFilterNames().contains(logName)) {
      if (!mEnabledFilters.contains(OmLog::filterName(OmLog::ALL_OMNISIM)))
        return;
    } else if (!mEnabledFilters.contains(OmLog::filterName(OmLog::ALL_CONTROLLERS)))
      return;
  }

  // check enabled levels
  if (!mEnabledLevels.contains(OmLog::filterName(OmLog::ALL))) {
    switch (level) {
      case OmLog::DEBUG:
      case OmLog::WARNING:
        if (!mEnabledLevels.contains(OmLog::levelName(OmLog::WARNING)))
          return;
        break;
      case OmLog::STDOUT:
      case OmLog::STDERR:
      case OmLog::INFO:
      case OmLog::ERROR:
        if (!mEnabledLevels.contains(OmLog::levelName(level)))
          return;
        break;
      case OmLog::FATAL:
      default:
        break;
    }
  }

  switch (level) {
    case OmLog::INFO:
    case OmLog::DEBUG:
      handlePossibleAnsiEscapeSequences(message, level);
      if (popup)
        OmMessageBox::info(message, this);
      break;
    case OmLog::WARNING:
    case OmLog::ERROR:
      handlePossibleAnsiEscapeSequences(message, level);
      if (popup)
        OmMessageBox::warning(message, this);
      break;
    case OmLog::STDOUT:
      handlePossibleAnsiEscapeSequences(message, level);
      break;
    case OmLog::STDERR:
      handlePossibleAnsiEscapeSequences(message, level);
      break;
    case OmLog::FATAL:
      handlePossibleAnsiEscapeSequences(message, level);
      if (popup)
        OmMessageBox::critical(message, this);
      break;
    default:
      break;
  }
}

QRegularExpression **OmConsole::createErrorMatchingPatterns() const {
  static QRegularExpression *exps[] = {
    // gcc: "e-puck.c:7:20: error: stdio.h : No such file or directory"
    // gcc: "main.cc:7: error: 'WbMainWin' was not declared in this scope"
    new QRegularExpression("(.+\\.\\w+):(\\d+):(\\d+):.*(?:\\w+):.*"),
    new QRegularExpression("(.+\\.\\w+):(\\d+):.*(?:\\w+):.*"),

    // Python: "  File "/nao_python/nao_python.py", line 304, in printFootSensors"
    new QRegularExpression(".*File \"(.+\\.py)\", line (\\d+).*"),

    // OmniSim parser: "ERROR: '/home/yvan/develop/webots/resources/projects/default/worlds/empty.wbt':19:2: error: skipped
    // unknown 'blabla' field in PointLight node"
    new QRegularExpression("ERROR: \'(.+\\.(?:wbt|proto))\':(\\d+):(\\d+): .*"),
    new QRegularExpression("ERROR: \'(.+\\.(?:wbt|proto))\':(\\d+): .*"),
    new QRegularExpression("ERROR: \'(.+\\.(?:wbt|proto))\': .*"),

    // terminate list
    NULL};

  return exps;
}

void OmConsole::updateTitle() {
  setObjectName(mConsoleName + mEnabledFilters.join(QString()) + mEnabledLevels.join(QString()));
  QString title(mConsoleName + " - ");
  title += mEnabledFilters.join(" | ");
  if (!mEnabledLevels.contains(OmLog::filterName(OmLog::ALL)))
    title += QString(" - ") + mEnabledLevels.join(" | ");
  setWindowTitle(title);
  if (mEnabledFilters.size() == 1 && mConsoleName == "Console")
    setTabbedTitle(mEnabledFilters.at(0));
  else
    setTabbedTitle(mConsoleName);
}

void OmConsole::closeEvent(QCloseEvent *event) {
  OmDockWidget::closeEvent(event);
  emit closed();
}

void OmConsole::updateFont() {
  // use the font of the preferences
  const OmPreferences *const prefs = OmPreferences::instance();
  QFont font;
  font.fromString(prefs->value("Editor/font").toString());
  mEditor->setFont(font);
}

void OmConsole::handleUserCommand(OmAction::OmActionKind actionKind) {
  switch (actionKind) {
    case OmAction::COPY:
      mEditor->copy();
      break;
    case OmAction::SELECT_ALL:
      mEditor->selectAll();
      break;
    case OmAction::FIND:
      openFindDialog();
      break;
    case OmAction::FIND_NEXT:
      if (mFindDialog != NULL)
        mFindDialog->next();
      else
        OmFindReplaceDialog::findNext(mTextFind, this);
      break;
    case OmAction::FIND_PREVIOUS:
      if (mFindDialog != NULL)
        mFindDialog->previous();
      else
        OmFindReplaceDialog::findPrevious(mTextFind, this);
      break;
    default:
      break;
  }
}

void OmConsole::enableCopyAction(bool enabled) {
  OmActionManager::instance()->setEnabled(OmAction::COPY, enabled);
}

void OmConsole::openFindDialog() {
  bool isNew = false;
  if (mFindDialog == NULL) {
    mFindDialog = new OmFindReplaceDialog(mTextFind, false, tr("Console"), this);
    connect(mFindDialog, &OmFindReplaceDialog::finished, this, &OmConsole::deleteFindDialog);
    isNew = true;
  }

  QTextCursor cur = mEditor->textCursor();
  if (cur.hasSelection() && cur.block() == mEditor->document()->findBlock(cur.anchor())) {
    QString selectedText = cur.selectedText();
    if (!selectedText.isEmpty())
      mFindDialog->setFindString(cur.selectedText());
  }

  mFindDialog->show();
  mFindDialog->raise();
  mFindDialog->activateWindow();
  if (isNew)
    mFindDialog->move(mFindDialog->pos() - QPoint(100, -100));
}

void OmConsole::deleteFindDialog() {
  // OmFindReplaceDialog deletes automatically on close
  mFindDialog = NULL;
}

void OmConsole::enableFilter(const QString &filter) {
  assert(!mEnabledFilters.contains(filter));
  mEnabledFilters.append(filter);
  updateTitle();
}

void OmConsole::disableFilter(const QString &filter) {
  mEnabledFilters.removeAll(filter);
  updateTitle();
}

void OmConsole::enableLevel(const QString &level) {
  assert(!mEnabledLevels.contains(level));
  mEnabledLevels.append(level);
  updateTitle();
}

void OmConsole::disableLevel(const QString &level) {
  mEnabledLevels.removeAll(level);
  updateTitle();
}
