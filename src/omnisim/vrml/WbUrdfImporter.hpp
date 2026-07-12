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

#ifndef WB_URDF_IMPORTER_HPP
#define WB_URDF_IMPORTER_HPP

//
// Description: parses URDF (Unified Robot Description Format) XML files
//   and converts them into Webots VRML node tree text. This enables
//   native URDF support: URDFRobot { url "robot.urdf" } blocks in world
//   files are expanded into Robot { ... } subtrees at world-load time.
//

#include <QtCore/QString>

class WbUrdfImporter {
public:
  // Convert a URDF file into VRML text representing a Webots Robot node.
  // Returns an empty QString on parse error and reports the error via WbLog.
  // The returned text starts with "Robot {" and ends with "}\n".
  static QString convertFile(const QString &urdfPath);

  // Convert raw URDF XML content (already loaded) into VRML.
  static QString convertContent(const QByteArray &xmlContent, const QString &sourceLabel);

  // Preprocess a world file's contents: find any URDFRobot { ... } blocks,
  // load the referenced URDF files, and replace each block with the expanded
  // Robot { ... } VRML. Returns the rewritten contents.
  // Translation, rotation, name, and controller fields written inside the
  // URDFRobot block are merged into the generated Robot node.
  static QByteArray expandUrdfRobotBlocks(const QByteArray &worldContents,
                                          const QString &worldFilePath);
};

#endif
