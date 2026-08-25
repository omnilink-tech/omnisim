// Copyright 2026 OmniLink
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

#ifndef OM_PERSPECTIVE_FILE_FORMAT_HPP
#define OM_PERSPECTIVE_FILE_FORMAT_HPP

//
// Description: the OmniSim perspective-file extension policy -- DUAL-READ, SINGLE-WRITE.
//
//   A perspective file holds the per-world user-interface state -- viewpoint,
//   dock and toolbar layout, maximized panel, open editor tabs, robot-window
//   visibility, optional renderings. The engine writes it next to the world as
//   a HIDDEN sibling named after the world's stem:
//
//       worlds/my_world.omniworld  ->  worlds/.my_world.omniperspective
//
//   OmniSim reads BOTH `.omniperspective` and `.wbproj`, indefinitely. An
//   existing `.wbproj` keeps working forever: dozens are tracked in this tree,
//   external projects and forks hold many more, and nobody may lose a saved
//   layout to an upgrade.
//
//   OmniSim WRITES only `.omniperspective`. Saving a world emits the new
//   extension beside it. A legacy sibling left over from an older release is
//   NOT deleted -- it simply stops being read, because the new extension is
//   preferred whenever both exist.
//
//   Why a new extension at all: `.wbproj` means "OmniSim", and it was the last
//   legacy-named extension OmniSim still wrote. The content has forked with the
//   engine too -- the file header migrated `Webots Project File version ...` ->
//   `OmniSim Project File version ...` while the reader kept accepting both
//   (see OmPerspective::load).
//
//   This is deliberately the SAME policy, with the same shape, as the world-file
//   one in OmWorldFileFormat.hpp. Read that header first; this is its sibling,
//   not a second opinion.
//
//   EVERY extension test must go through this header. Do not hand-roll
//   `endsWith(".wbproj")` -- that is how half a dual-read policy ships.
//

#include <QtCore/QFileInfo>
#include <QtCore/QString>
#include <QtCore/QStringList>

namespace OmPerspectiveFileFormat {

  // The one extension OmniSim writes. Includes the leading dot.
  inline const QString &writeExtension() {
    static const QString ext(".omniperspective");
    return ext;
  }

  // The legacy extension, still read forever. Includes the leading dot.
  inline const QString &legacyExtension() {
    static const QString ext(".wbproj");
    return ext;
  }

  // Every extension accepted on read, preferred first.
  inline const QStringList &readExtensions() {
    static const QStringList exts = QStringList() << writeExtension() << legacyExtension();
    return exts;
  }

  // Suffixes without the leading dot -- for QFileInfo::suffix() comparisons.
  inline const QStringList &readSuffixes() {
    static const QStringList sfx = QStringList() << "omniperspective"
                                                 << "wbproj";
    return sfx;
  }

  // true if `path` names a perspective file in either accepted extension.
  inline bool isPerspectiveFile(const QString &path) {
    foreach (const QString &ext, readExtensions()) {
      if (path.endsWith(ext, Qt::CaseInsensitive))
        return true;
    }
    return false;
  }

  // true if `suffix` (no leading dot, e.g. from QFileInfo::suffix()) is a
  // perspective suffix.
  inline bool isPerspectiveSuffix(const QString &suffix) {
    foreach (const QString &sfx, readSuffixes()) {
      if (QString::compare(suffix, sfx, Qt::CaseInsensitive) == 0)
        return true;
    }
    return false;
  }

  // Directory name filters for QDir::setNameFilters() -- both extensions. The
  // leading dot is part of the filter because perspective files are always
  // written hidden; the QDir must also be set to list QDir::Hidden entries.
  inline const QStringList &nameFilters() {
    static const QStringList filters = QStringList() << ".*.omniperspective"
                                                     << ".*.wbproj";
    return filters;
  }

  // Drop whichever perspective extension `path` carries. Returns `path`
  // unchanged when it carries neither, so it is safe to call unconditionally.
  inline QString stripExtension(const QString &path) {
    foreach (const QString &ext, readExtensions()) {
      if (path.endsWith(ext, Qt::CaseInsensitive))
        return path.left(path.length() - ext.length());
    }
    return path;
  }

  // Force `path` to the write extension: converts a legacy `.wbproj` name and
  // appends to an extension-less one. The single funnel for "what do we save as".
  inline QString toWriteExtension(const QString &path) {
    if (path.endsWith(writeExtension(), Qt::CaseInsensitive))
      return path;
    if (path.endsWith(legacyExtension(), Qt::CaseInsensitive))
      return stripExtension(path) + writeExtension();
    return path + writeExtension();
  }

  // Resolve a perspective path that may be named with either extension: returns
  // the path as given when it exists, otherwise the sibling in the other
  // extension if THAT exists, otherwise the path as given. Call it with the
  // WRITE-extension name and it degrades to the legacy file only when the
  // current one is absent -- which is the whole of the dual-read rule, and the
  // reason an upgrade never loses a saved layout.
  inline QString resolveExisting(const QString &path) {
    if (path.isEmpty() || QFileInfo::exists(path))
      return path;
    foreach (const QString &ext, readExtensions()) {
      if (!path.endsWith(ext, Qt::CaseInsensitive))
        continue;
      const QString stem = path.left(path.length() - ext.length());
      foreach (const QString &alt, readExtensions()) {
        if (alt == ext)
          continue;
        if (QFileInfo::exists(stem + alt))
          return stem + alt;
      }
      break;
    }
    return path;
  }

}  // namespace OmPerspectiveFileFormat

#endif
